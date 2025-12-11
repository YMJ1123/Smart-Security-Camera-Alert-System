import sys
sys.path.append("/usr/lib/python3/dist-packages")  # 使用系統的 picamera2

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time
from datetime import datetime
import os

# -------- 新增：PCA9685 相關函式庫 --------
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
# ----------------------------------------

# 引入原本的 Buzzer 與 Email 模組
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off
from send_email import send_email_with_latest_image

# 事件狀態常數
NO_PERSON = 0
PERSON_DETECTED = 1

# 警報閾值
ENTER_THRESHOLD = 5   # 連續 5 幀有人 -> 確認入侵
EXIT_THRESHOLD = 8    # 連續 8 幀沒人 -> 離開

# 截圖目錄
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)

# ====== 重要設定 ======
# 設為 True 關閉預覽視窗，大幅提升 FPS 和性能
HEADLESS_MODE = True


def save_screenshot(img):
    """存圖片到 invade_image/，檔名使用 timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] Screenshot saved: {filepath}")
    return filepath


class PIDController:
    """PID 控制器，用於計算伺服馬達的修正量"""
    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit or (output_limit * 10 if output_limit else 100)
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.prev_output = 0.0

    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: 
            dt = 1e-16

        # P term
        p_term = self.kp * error
        
        # I term (抗積分飽和)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral
        
        # D term
        raw_derivative = (error - self.prev_error) / dt
        d_term = self.kd * raw_derivative

        output = p_term + i_term + d_term

        # 更新狀態
        self.prev_error = error
        self.last_time = current_time

        # 限制輸出範圍
        if self.output_limit:
            output = max(min(output, self.output_limit), -self.output_limit)
        
        self.prev_output = output
        return output

    def reset(self):
        """重置 PID 狀態"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.prev_output = 0.0


class HumanTrackingCamera:
    """結合人體偵測與 PCA9685 伺服馬達追蹤的相機系統"""
    def __init__(self, width=640, height=480):
        # 畫面參數
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # 伺服馬達角度 (90度為中心)
        self.pan_angle = 90.0   # 水平 (左右)
        self.tilt_angle = 90.0  # 垂直 (上下)
        
        # 平滑化角度 (減少抖動)
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.smooth_factor = 0.2  # 平滑係數
        
        # 上次發送的角度
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        self.min_angle_change = 0.5  # 最小變動量
        
        # 更新頻率控制 - PCA9685 硬體 PWM
        self.servo_update_interval = 0.05  # 50ms 更新一次
        self.last_servo_update_time = 0
        
        # PID 參數 (平衡版：比原本快，但穩定)
        # Kp: 0.05 (比例增益)
        # Ki: 0 (關閉積分，避免累積問題)
        # Kd: 0.008 (微分增益)
        # output_limit: 3.0 (調整量限制)
        self.pid_pan = PIDController(kp=0.05, ki=0, kd=0.008, output_limit=3.0, integral_limit=10)
        self.pid_tilt = PIDController(kp=0.05, ki=0, kd=0.008, output_limit=3.0, integral_limit=10)
        
        # 死區 (誤差小於此值不動作)
        self.dead_zone = 20
        
        # -------- 初始化 PCA9685 與 Servos --------
        self.servo_pan = None
        self.servo_tilt = None
        self.pca = None
        
        try:
            i2c = board.I2C()  # 使用板子上的 SDA/SCL
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50  # 伺服馬達標準頻率
            
            # 設定 Channel (請依實際接線修改)
            # Channel 0 -> Pan (左右)
            # Channel 1 -> Tilt (上下)
            # min_pulse/max_pulse 可根據你的馬達型號微調 (通常 SG90 為 500-2500)
            self.servo_pan = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500)
            self.servo_tilt = servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500)
            
            # 初始化歸中
            self.servo_pan.angle = self.pan_angle
            self.servo_tilt.angle = self.tilt_angle
            print(f"[SERVO] PCA9685 initialized. Pan=Ch0, Tilt=Ch1")
            
        except Exception as e:
            print(f"[ERROR] Failed to initialize PCA9685: {e}")
            self.servo_pan = None
            self.servo_tilt = None
        # ----------------------------------------
        
        # 追蹤狀態
        self.tracking_active = False
        self.last_detection_time = time.time()
        
        # 警報狀態
        self.alert_state = NO_PERSON
        self.detect_count = 0
        self.nodetect_count = 0
        
    def find_largest_person(self, results):
        """從 YOLO 結果中找出最大的人"""
        if len(results[0].boxes) == 0:
            return None
        
        boxes = results[0].boxes
        max_area = 0
        largest_box = None
        
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                largest_box = (x1, y1, x2, y2)
        
        return largest_box
    
    def get_box_center(self, box):
        if box is None:
            return None, None
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
    
    def calculate_new_angles(self, target_x, target_y):
        """計算新的伺服馬達角度 (含 PID 與平滑化)"""
        if target_x is None or target_y is None:
            return self.smoothed_pan, self.smoothed_tilt
        
        # 1. 計算誤差
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y
        
        # 2. 死區判斷
        if abs(error_x) < self.dead_zone:
            error_x = 0
        if abs(error_y) < self.dead_zone:
            error_y = 0
        
        # 3. PID 計算修正量
        delta_pan = self.pid_pan.compute(error_x)
        delta_tilt = self.pid_tilt.compute(error_y)
        
        # 4. 更新目標角度
        # Pan: 視目標在左(error>0)需左轉(角度增加)
        self.pan_angle += delta_pan
        # Tilt: 視目標在上(error>0)需上抬(角度增加)
        self.tilt_angle += delta_tilt
        
        # 5. 限制角度範圍 (0~180)
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))
        
        # 6. 低通濾波 (平滑化移動)
        self.smoothed_pan = self.smoothed_pan + self.smooth_factor * (self.pan_angle - self.smoothed_pan)
        self.smoothed_tilt = self.smoothed_tilt + self.smooth_factor * (self.tilt_angle - self.smoothed_tilt)
        
        return self.smoothed_pan, self.smoothed_tilt
    
    def set_servo_angles(self, pan, tilt):
        """設定伺服馬達角度"""
        current_time = time.time()
        
        # 檢查是否達到更新時間間隔
        if current_time - self.last_servo_update_time < self.servo_update_interval:
            return
        
        try:
            # Pan 控制
            if self.servo_pan is not None:
                # 只有變化夠大才更新，或是強制刷新
                if abs(pan - self.last_sent_pan) >= self.min_angle_change:
                    # 確保角度在 0-180 之間
                    safe_pan = max(0, min(180, pan))
                    self.servo_pan.angle = safe_pan
                    self.last_sent_pan = safe_pan
            
            # Tilt 控制
            if self.servo_tilt is not None:
                if abs(tilt - self.last_sent_tilt) >= self.min_angle_change:
                    safe_tilt = max(0, min(180, tilt))
                    self.servo_tilt.angle = safe_tilt
                    self.last_sent_tilt = safe_tilt
            
            self.last_servo_update_time = current_time
        except Exception as e:
            print(f"[ERROR] Failed to set servo angles: {e}")
    
    def reset_to_center(self):
        """回歸中心位置"""
        print("[TRACKING] Resetting to center position...")
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        
        try:
            if self.servo_pan is not None:
                self.servo_pan.angle = 90.0
            if self.servo_tilt is not None:
                self.servo_tilt.angle = 90.0
        except Exception as e:
            print(f"[ERROR] Failed to reset servos: {e}")
        
        self.pid_pan.reset()
        self.pid_tilt.reset()
        self.tracking_active = False
        self.last_detection_time = time.time()
    
    def update_alert_state(self, person_found, frame=None):
        """更新警報狀態"""
        if person_found:
            self.detect_count += 1
            self.nodetect_count = 0
        else:
            self.nodetect_count += 1
            self.detect_count = 0
        
        # 狀態轉換: NO_PERSON -> PERSON_DETECTED
        if self.alert_state == NO_PERSON and self.detect_count >= ENTER_THRESHOLD:
            print("[ALERT] Intruder detected! Buzzer ON")
            self.alert_state = PERSON_DETECTED
            
            beep_pattern(duration=0.1, repeat=3, interval=0.05)
            buzzer_on()
            
            if frame is not None:
                save_screenshot(frame)
                send_email_with_latest_image()
        
        # 狀態轉換: PERSON_DETECTED -> NO_PERSON
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] Intruder left. Buzzer OFF")
            
            if frame is not None:
                save_screenshot(frame)
            
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_tracking_info(self, frame, box, target_x, target_y):
        """繪製追蹤資訊"""
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        
        if box is not None and target_x is not None and target_y is not None:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                         (0, 255, 255), 3)
            cv2.circle(frame, (int(target_x), int(target_y)), 8, (0, 0, 255), -1)
            cv2.line(frame, (self.center_x, self.center_y), 
                    (int(target_x), int(target_y)), (255, 0, 255), 2)
            
            error_x = self.center_x - target_x
            error_y = self.center_y - target_y
            cv2.putText(frame, f"Error: X={error_x:.1f}, Y={error_y:.1f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Pan: {self.pan_angle:.1f} deg", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Tilt: {self.tilt_angle:.1f} deg", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        status = "TRACKING" if self.tracking_active else "SEARCHING"
        color = (0, 255, 0) if self.tracking_active else (0, 165, 255)
        cv2.putText(frame, f"Status: {status}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        alert_status = "ALARM ON" if self.alert_state == PERSON_DETECTED else "STANDBY"
        alert_color = (0, 0, 255) if self.alert_state == PERSON_DETECTED else (128, 128, 128)
        cv2.putText(frame, f"Alert: {alert_status}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, alert_color, 2)
        
        return frame


def main():
    print("=" * 50)
    print("Human Tracking Camera System (PCA9685 Version)")
    print("Uses YOLO to detect humans and PCA9685 Driver for smooth tracking")
    print("=" * 50)
    print(f"HEADLESS_MODE = {HEADLESS_MODE}")
    if HEADLESS_MODE:
        print("  (No preview window - better performance)")
        print("  Press Ctrl+C to exit")
    else:
        print("Controls:")
        print("  q - Exit program")
        print("  r - Reset servos to center position")
    print("=" * 50)
    
    # 初始化 Buzzer
    init_buzzer()
    
    # 初始化 YOLO
    print("[INIT] Loading YOLO model...")
    model = YOLO("yolo11n.pt")
    
    # 初始化相機
    print("[INIT] Initializing camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=libcamera.Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    # 初始化追蹤系統
    print("[INIT] Initializing tracking system...")
    tracker = HumanTrackingCamera(width=640, height=480)
    
    print("[READY] System ready! Starting tracking...")
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    # 追蹤超時設定 (秒) - 沒人多久後回正
    RETURN_TO_CENTER_TIMEOUT = 3.0
    
    try:
        while True:
            frame = picam2.capture_array()
            frame_count += 1
            
            # YOLO 偵測
            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            
            largest_box = tracker.find_largest_person(results)
            person_found = largest_box is not None
            
            # 只在需要截圖時才生成 annotated frame (節省 CPU)
            annotated = None
            if tracker.alert_state == NO_PERSON and tracker.detect_count >= ENTER_THRESHOLD - 1:
                # 即將觸發警報，準備截圖
                annotated = results[0].plot()
            elif tracker.alert_state == PERSON_DETECTED and tracker.nodetect_count >= EXIT_THRESHOLD - 1:
                # 即將結束警報，準備截圖
                annotated = results[0].plot()
            
            # 更新警報
            tracker.update_alert_state(person_found, annotated if annotated else frame)
            
            if largest_box is not None:
                # 發現目標 -> 追蹤
                target_x, target_y = tracker.get_box_center(largest_box)
                pan, tilt = tracker.calculate_new_angles(target_x, target_y)
                tracker.set_servo_angles(pan, tilt)
                
                tracker.tracking_active = True
                tracker.last_detection_time = time.time()
                
            else:
                # 沒發現目標
                target_x, target_y = None, None
                
                # 超時回正
                time_since_last_detection = time.time() - tracker.last_detection_time
                if time_since_last_detection > RETURN_TO_CENTER_TIMEOUT and tracker.tracking_active:
                    print(f"[TRACKING] No person detected for {RETURN_TO_CENTER_TIMEOUT:.1f}s, returning to center...")
                    tracker.reset_to_center()
            
            # 計算並顯示 FPS
            if frame_count % 30 == 0:
                current_time = time.time()
                fps = 30 / (current_time - fps_time)
                fps_time = current_time
                print(f"[INFO] FPS: {fps:.1f} | Pan: {tracker.smoothed_pan:.1f}° | Tilt: {tracker.smoothed_tilt:.1f}° | Tracking: {tracker.tracking_active}")
            
            # 只在非 headless 模式顯示視窗
            if not HEADLESS_MODE:
                # 生成顯示用的 frame
                if annotated is None:
                    annotated = results[0].plot()
                display_frame = tracker.draw_tracking_info(annotated, largest_box, 
                                                           target_x, target_y)
                cv2.putText(display_frame, f"FPS: {fps:.1f}", 
                           (display_frame.shape[1] - 150, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                big = cv2.resize(display_frame, (1280, 960))
                cv2.imshow("Human Tracking Camera (PCA9685)", big)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    tracker.reset_to_center()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Keyboard interrupt detected...")
    
    finally:
        print("[CLEANUP] Cleaning up...")
        buzzer_off()
        cleanup_buzzer()
        tracker.reset_to_center()
        
        # PCA9685 Cleanup
        if tracker.pca:
            try:
                tracker.pca.deinit()
            except:
                pass
                
        picam2.close()
        if not HEADLESS_MODE:
            cv2.destroyAllWindows()
        print("[EXIT] Clean exit. Goodbye!")


if __name__ == "__main__":
    main()