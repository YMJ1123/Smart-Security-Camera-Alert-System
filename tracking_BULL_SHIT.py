import sys
sys.path.append("/usr/lib/python3/dist-packages")  # Use system's picamera2

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time
from datetime import datetime
import os

# Buzzer module
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off

# Email module
from send_email import send_email_with_latest_image

# Event state constants
NO_PERSON = 0
PERSON_DETECTED = 1

# Alert thresholds
ENTER_THRESHOLD = 5
EXIT_THRESHOLD = 8

# Screenshot directory
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)

# GPIO Servo control
from gpiozero import AngularServo

# Servo GPIO pins
SERVO_PAN_PIN = 18    # GPIO18 - Pan (Horizontal)
SERVO_TILT_PIN = 23   # GPIO23 - Tilt (Vertical)

# Tracking timeout
RETURN_TO_CENTER_TIMEOUT = 3.0


class PIDController:
    """PID controller (Untouched)"""
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
        if dt <= 0: dt = 1e-16

        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral
        raw_derivative = (error - self.prev_error) / dt
        d_term = self.kd * raw_derivative

        output = p_term + i_term + d_term
        self.prev_error = error
        self.last_time = current_time

        if self.output_limit:
            output = max(min(output, self.output_limit), -self.output_limit)
        
        self.prev_output = output
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.prev_output = 0.0


class HumanTrackingCamera:
    """Camera system combining human detection with servo tracking"""
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Initialize servo angles
        self.pan_angle = 90.0   
        self.tilt_angle = 90.0  
        
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.smooth_factor = 0.12
        
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        self.min_angle_change = 0.5
        
        self.servo_update_interval = 3
        self.last_servo_update_time = 0
        
        self.pid_pan = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        self.pid_tilt = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        
        self.dead_zone = 35
        
        self.servo_pan = None
        self.servo_tilt = None
        try:
            self.servo_pan = AngularServo(
                SERVO_PAN_PIN,
                min_angle=0,
                max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
            )
            self.servo_tilt = AngularServo(
                SERVO_TILT_PIN,
                min_angle=0,
                max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
            )
            self.servo_pan.angle = self.pan_angle
            self.servo_tilt.angle = self.tilt_angle
            print(f"[SERVO] GPIO Servos initialized.")
        except Exception as e:
            print(f"[ERROR] Failed to initialize GPIO servos: {e}")
        
        self.tracking_active = False
        self.last_detection_time = time.time()
        
        self.alert_state = NO_PERSON
        self.detect_count = 0
        self.nodetect_count = 0
        
    def find_largest_person(self, results):
        if len(results[0].boxes) == 0: return None
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
        if box is None: return None, None
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
    
    def calculate_new_angles(self, target_x, target_y):
        if target_x is None or target_y is None:
            return self.smoothed_pan, self.smoothed_tilt
        
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y
        
        if abs(error_x) < self.dead_zone: error_x = 0
        if abs(error_y) < self.dead_zone: error_y = 0
        
        delta_pan = self.pid_pan.compute(error_x)
        delta_tilt = self.pid_tilt.compute(error_y)
        
        self.pan_angle += delta_pan
        self.tilt_angle += delta_tilt
        
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))
        
        self.smoothed_pan = self.smoothed_pan + self.smooth_factor * (self.pan_angle - self.smoothed_pan)
        self.smoothed_tilt = self.smoothed_tilt + self.smooth_factor * (self.tilt_angle - self.smoothed_tilt)
        
        return self.smoothed_pan, self.smoothed_tilt
    
    def set_servo_angles(self, pan, tilt, force=False):
        current_time = time.time()
        if not force and (current_time - self.last_servo_update_time < self.servo_update_interval):
            return
        
        try:
            if self.servo_pan is not None:
                if force or abs(pan - self.last_sent_pan) >= self.min_angle_change:
                    self.servo_pan.angle = pan
                    self.last_sent_pan = pan
            
            if self.servo_tilt is not None:
                if force or abs(tilt - self.last_sent_tilt) >= self.min_angle_change:
                    self.servo_tilt.angle = tilt
                    self.last_sent_tilt = tilt
            
            self.last_servo_update_time = current_time
        except Exception as e:
            print(f"[ERROR] Failed to set servo angles: {e}")

    # --- [關鍵修改] 解決抖動問題的巡邏邏輯 ---
    def run_patrol_routine(self):
        """
        策略：動 -> 等待到達 -> Detach(斷訊號) -> 休息
        """
        print("[PATROL] Starting 'Move & Detach' Patrol...")
        self.tracking_active = False 
        
        if self.servo_pan is None: return

        # Helper function: 轉動並立刻放鬆
        def move_and_relax(servo, angle, wait_time=2.0):
            print(f" -> Moving to {angle}...")
            servo.angle = angle
            time.sleep(0.5)  # 給它 0.5 秒轉過去
            servo.detach()   # [關鍵] 到達後立刻斷開訊號，防止抖動
            time.sleep(wait_time - 0.5) # 剩下的時間安靜等待

        # 1. 回到中間 (90度)
        move_and_relax(self.servo_pan, 90, wait_time=2.0)

        # 2. 轉到 0度
        move_and_relax(self.servo_pan, 0, wait_time=2.0)

        # 3. 轉到 180度
        move_and_relax(self.servo_pan, 180, wait_time=2.0)

        # 4. 回歸中間
        move_and_relax(self.servo_pan, 90, wait_time=1.0)

        print("[PATROL] Complete.")
        
        # 確保最後是斷開的
        if self.servo_pan: self.servo_pan.detach()
        if self.servo_tilt: self.servo_tilt.detach()

        # 更新狀態
        self.pan_angle = 90.0
        self.smoothed_pan = 90.0
        self.last_sent_pan = 90.0
    
    def reset_to_center(self):
        print("[TRACKING] Resetting to center...")
        if self.servo_pan: 
            self.servo_pan.angle = 90.0
            time.sleep(0.5)
            self.servo_pan.detach() # 歸位後也斷開
        
        if self.servo_tilt: 
            self.servo_tilt.angle = 90.0
            time.sleep(0.5)
            self.servo_tilt.detach() # 歸位後也斷開

        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        
        self.pid_pan.reset()
        self.pid_tilt.reset()
        self.tracking_active = False
        self.last_detection_time = time.time()
    
    def update_alert_state(self, person_found, frame=None):
        if person_found:
            self.detect_count += 1
            self.nodetect_count = 0
        else:
            self.nodetect_count += 1
            self.detect_count = 0
        
        if self.alert_state == NO_PERSON and self.detect_count >= ENTER_THRESHOLD:
            print("[ALERT] Intruder detected! Buzzer ON")
            self.alert_state = PERSON_DETECTED
            beep_pattern(duration=0.1, repeat=3, interval=0.05)
            buzzer_on()
            if frame is not None:
                save_screenshot(frame)
                send_email_with_latest_image()
        
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] Intruder left. Buzzer OFF")
            if frame is not None:
                save_screenshot(frame)
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_tracking_info(self, frame, box, target_x, target_y):
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
    print("Human Tracking Camera System (Jitter Fix Edition)")
    print("=" * 50)
    print("Controls:")
    print("  q - Exit")
    print("  r - Reset Center")
    print("  p - Run Test Pattern (Move & Detach Strategy)")
    print("=" * 50)
    
    init_buzzer()
    model = YOLO("yolo11n.pt")
    
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=libcamera.Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    
    tracker = HumanTrackingCamera(width=640, height=480)
    
    print("[READY] System ready!")
    print()
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    try:
        while True:
            frame = picam2.capture_array()
            frame_count += 1
            
            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            largest_box = tracker.find_largest_person(results)
            person_found = largest_box is not None
            
            annotated = results[0].plot()
            tracker.update_alert_state(person_found, annotated)
            
            if largest_box is not None:
                target_x, target_y = tracker.get_box_center(largest_box)
                pan, tilt = tracker.calculate_new_angles(target_x, target_y)
                tracker.set_servo_angles(pan, tilt)
                tracker.tracking_active = True
                tracker.last_detection_time = time.time()
            else:
                target_x, target_y = None, None
                time_since_last_detection = time.time() - tracker.last_detection_time
                if time_since_last_detection > RETURN_TO_CENTER_TIMEOUT and tracker.tracking_active:
                    tracker.reset_to_center()
            
            display_frame = tracker.draw_tracking_info(annotated, largest_box, target_x, target_y)
            
            if frame_count % 30 == 0:
                current_time = time.time()
                fps = 30 / (current_time - fps_time)
                fps_time = current_time
            
            cv2.putText(display_frame, f"FPS: {fps:.1f}", (display_frame.shape[1] - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            big = cv2.resize(display_frame, (1280, 960))
            cv2.imshow("Human Tracking Camera", big)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                tracker.reset_to_center()
            elif key == ord('p'):
                tracker.run_patrol_routine()
                tracker.pid_pan.reset()
                tracker.pid_tilt.reset()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Keyboard interrupt detected...")
    
    finally:
        print("[CLEANUP] Cleaning up...")
        buzzer_off()
        cleanup_buzzer()
        tracker.reset_to_center()
        try:
            if tracker.servo_pan: tracker.servo_pan.detach()
            if tracker.servo_tilt: tracker.servo_tilt.detach()
        except Exception:
            pass
        picam2.close()
        cv2.destroyAllWindows()
        print("[EXIT] Goodbye!")

if __name__ == "__main__":
    main()