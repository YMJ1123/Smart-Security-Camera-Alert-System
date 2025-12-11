#!/usr/bin/env python3
"""
Human Tracking Camera with PCA9685 Servo Control
- YOLO 人體偵測
- PCA9685 控制 Pan/Tilt 馬達追蹤
- Buzzer 警報
- Email 通知
"""

import sys
sys.path.append("/usr/lib/python3/dist-packages")

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time
from datetime import datetime
import os

# PCA9685 相關
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Buzzer 與 Email
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off
from send_email import send_email_with_latest_image

# ============ 設定參數 ============
# 顯示模式：True = 無視窗 (高效能), False = 有視窗 (方便調試)
HEADLESS = True

# 警報閾值
ENTER_THRESHOLD = 5   # 連續幾幀有人才觸發警報
EXIT_THRESHOLD = 8    # 連續幾幀沒人才關閉警報

# 追蹤參數
DEAD_ZONE = 25        # 死區 (像素)，誤差小於此值不動作
RETURN_TIMEOUT = 3.0  # 幾秒沒偵測到人就回中

# PID 參數 (簡化版，只用 P 控制)
KP = 0.15             # 比例增益，越大追蹤越快
MAX_STEP = 5.0        # 每幀最大調整角度

# 馬達更新頻率
SERVO_INTERVAL = 0.08  # 每 80ms 更新一次馬達

# 歸中角度 (根據雲台安裝位置調整)
CENTER_PAN = 90        # Pan 水平歸中角度
CENTER_TILT = 100       # Tilt 垂直歸中角度 (90太高的話改成60或70)

# 方向反轉 (如果馬達轉的方向相反，設為 True)
REVERSE_PAN = False     # 左右反轉
REVERSE_TILT = True    # 上下反轉
# ================================

# 狀態常數
NO_PERSON = 0
PERSON_DETECTED = 1

# 截圖目錄
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)


def save_screenshot(img):
    """存截圖"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] {filepath}")
    return filepath


class Tracker:
    """追蹤器：處理人體偵測、馬達控制、警報"""
    
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # 馬達角度
        self.pan = float(CENTER_PAN)    # 水平 (左右)
        self.tilt = float(CENTER_TILT)  # 垂直 (上下)
        
        # 馬達更新時間控制
        self.last_servo_time = 0
        
        # 追蹤狀態
        self.tracking = False
        self.last_seen_time = time.time()
        
        # 警報狀態
        self.alert_state = NO_PERSON
        self.detect_count = 0
        self.nodetect_count = 0
        
        # 初始化 PCA9685
        self.pca = None
        self.servo_pan = None
        self.servo_tilt = None
        self._init_servos()
    
    def _init_servos(self):
        """初始化 PCA9685 和伺服馬達"""
        try:
            i2c = board.I2C()
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50  # 50Hz for servo
            
            # Channel 0 = Pan (水平), Channel 1 = Tilt (垂直)
            self.servo_pan = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500)
            self.servo_tilt = servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500)
            
            # 歸中
            self.servo_pan.angle = CENTER_PAN
            self.servo_tilt.angle = CENTER_TILT
            print(f"[SERVO] PCA9685 初始化成功 (Pan=Ch0@{CENTER_PAN}°, Tilt=Ch1@{CENTER_TILT}°)")
        except Exception as e:
            print(f"[ERROR] PCA9685 初始化失敗: {e}")
    
    def find_largest_person(self, results):
        """找出最大的人"""
        if len(results[0].boxes) == 0:
            return None
        
        max_area = 0
        largest = None
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                largest = (x1, y1, x2, y2)
        return largest
    
    def get_center(self, box):
        """取得 bounding box 中心"""
        if box is None:
            return None, None
        x1, y1, x2, y2 = box
        return (x1 + x2) / 2, (y1 + y2) / 2
    
    def update_tracking(self, target_x, target_y):
        """更新追蹤角度"""
        if target_x is None or target_y is None:
            return
        
        # 計算誤差
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y
        
        # 死區
        if abs(error_x) < DEAD_ZONE:
            error_x = 0
        if abs(error_y) < DEAD_ZONE:
            error_y = 0
        
        # 簡單 P 控制
        delta_pan = KP * error_x
        delta_tilt = KP * error_y
        
        # 方向反轉
        if REVERSE_PAN:
            delta_pan = -delta_pan
        if REVERSE_TILT:
            delta_tilt = -delta_tilt
        
        # 限制每步最大調整量
        delta_pan = max(-MAX_STEP, min(MAX_STEP, delta_pan))
        delta_tilt = max(-MAX_STEP, min(MAX_STEP, delta_tilt))
        
        # 更新角度
        self.pan += delta_pan
        self.tilt += delta_tilt
        
        # 限制範圍 0-180
        self.pan = max(0, min(180, self.pan))
        self.tilt = max(0, min(180, self.tilt))
    
    def move_servos(self):
        """發送角度到馬達 (有頻率限制)"""
        now = time.time()
        if now - self.last_servo_time < SERVO_INTERVAL:
            return
        
        try:
            if self.servo_pan is not None:
                self.servo_pan.angle = self.pan
            if self.servo_tilt is not None:
                self.servo_tilt.angle = self.tilt
            self.last_servo_time = now
        except Exception as e:
            print(f"[ERROR] 馬達控制失敗: {e}")
    
    def reset_to_center(self):
        """馬達歸中"""
        print(f"[TRACKING] 歸中 (Pan={CENTER_PAN}°, Tilt={CENTER_TILT}°)...")
        self.pan = float(CENTER_PAN)
        self.tilt = float(CENTER_TILT)
        try:
            if self.servo_pan is not None:
                self.servo_pan.angle = CENTER_PAN
            if self.servo_tilt is not None:
                self.servo_tilt.angle = CENTER_TILT
        except:
            pass
        self.tracking = False
        self.last_seen_time = time.time()
    
    def update_alert(self, person_found, frame=None):
        """更新警報狀態"""
        if person_found:
            self.detect_count += 1
            self.nodetect_count = 0
        else:
            self.nodetect_count += 1
            self.detect_count = 0
        
        # 觸發警報
        if self.alert_state == NO_PERSON and self.detect_count >= ENTER_THRESHOLD:
            print("[ALERT] 偵測到入侵者! 警報開啟")
            self.alert_state = PERSON_DETECTED
            beep_pattern(duration=0.1, repeat=3, interval=0.05)
            buzzer_on()
            if frame is not None:
                save_screenshot(frame)
                send_email_with_latest_image()
        
        # 關閉警報
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] 入侵者離開，警報關閉")
            if frame is not None:
                save_screenshot(frame)
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_info(self, frame, box, tx, ty):
        """畫追蹤資訊"""
        # 中心十字
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        
        if box is not None and tx is not None:
            x1, y1, x2, y2 = box
            # 目標框
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 3)
            # 目標中心
            cv2.circle(frame, (int(tx), int(ty)), 8, (0, 0, 255), -1)
            # 連線
            cv2.line(frame, (self.center_x, self.center_y), (int(tx), int(ty)), (255, 0, 255), 2)
            # 誤差
            cv2.putText(frame, f"Error: ({self.center_x-tx:.0f}, {self.center_y-ty:.0f})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 角度
        cv2.putText(frame, f"Pan: {self.pan:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Tilt: {self.tilt:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 狀態
        status = "TRACKING" if self.tracking else "SEARCHING"
        color = (0, 255, 0) if self.tracking else (0, 165, 255)
        cv2.putText(frame, status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 警報
        alert = "ALARM!" if self.alert_state == PERSON_DETECTED else "STANDBY"
        acolor = (0, 0, 255) if self.alert_state == PERSON_DETECTED else (128, 128, 128)
        cv2.putText(frame, alert, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, acolor, 2)
        
        return frame
    
    def cleanup(self):
        """清理資源"""
        try:
            if self.servo_pan is not None:
                self.servo_pan.angle = CENTER_PAN
            if self.servo_tilt is not None:
                self.servo_tilt.angle = CENTER_TILT
            if self.pca is not None:
                self.pca.deinit()
        except:
            pass


def main():
    print("=" * 50)
    print("Human Tracking with PCA9685")
    print("=" * 50)
    print(f"HEADLESS = {HEADLESS}")
    if HEADLESS:
        print("(無視窗模式，按 Ctrl+C 結束)")
    else:
        print("按 q 結束, r 歸中")
    print("=" * 50)
    
    # 初始化
    print("[INIT] Buzzer...")
    init_buzzer()
    
    print("[INIT] YOLO...")
    model = YOLO("yolo11n.pt")
    
    print("[INIT] Camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=libcamera.Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    print("[INIT] Tracker...")
    tracker = Tracker(640, 480)
    
    print("[READY] 開始追蹤!")
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    try:
        while True:
            frame = picam2.capture_array()
            frame_count += 1
            
            # YOLO 偵測
            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            box = tracker.find_largest_person(results)
            person_found = box is not None
            
            # 更新警報 (只在需要時生成 annotated frame)
            if tracker.alert_state == NO_PERSON and tracker.detect_count >= ENTER_THRESHOLD - 1:
                annotated = results[0].plot()
                tracker.update_alert(person_found, annotated)
            elif tracker.alert_state == PERSON_DETECTED and tracker.nodetect_count >= EXIT_THRESHOLD - 1:
                annotated = results[0].plot()
                tracker.update_alert(person_found, annotated)
            else:
                tracker.update_alert(person_found, frame)
            
            # 追蹤
            if person_found:
                tx, ty = tracker.get_center(box)
                tracker.update_tracking(tx, ty)
                tracker.move_servos()
                tracker.tracking = True
                tracker.last_seen_time = time.time()
            else:
                tx, ty = None, None
                # 超時歸中
                if time.time() - tracker.last_seen_time > RETURN_TIMEOUT and tracker.tracking:
                    tracker.reset_to_center()
            
            # FPS
            if frame_count % 30 == 0:
                now = time.time()
                fps = 30 / (now - fps_time)
                fps_time = now
                print(f"[INFO] FPS: {fps:.1f} | Pan: {tracker.pan:.1f} | Tilt: {tracker.tilt:.1f} | Track: {tracker.tracking}")
            
            # 顯示
            if not HEADLESS:
                annotated = results[0].plot()
                display = tracker.draw_info(annotated, box, tx, ty)
                cv2.putText(display, f"FPS: {fps:.1f}", (530, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("Tracking", display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    tracker.reset_to_center()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    
    finally:
        print("[CLEANUP]...")
        buzzer_off()
        cleanup_buzzer()
        tracker.reset_to_center()
        tracker.cleanup()
        picam2.close()
        if not HEADLESS:
            cv2.destroyAllWindows()
        print("[EXIT] Done!")


if __name__ == "__main__":
    main()

