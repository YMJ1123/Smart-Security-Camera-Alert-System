#!/usr/bin/env python3
import sys
sys.path.append("/usr/lib/python3/dist-packages")

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time
from datetime import datetime
import os

# Hardware dependencies
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Custom modules
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off
from send_email import send_email_with_latest_image
from log import init_log, log_event

# ============ Configuration ============
# Display mode: True = Headless (Performance), False = Windowed (Debug)
HEADLESS = False

# Alert Thresholds (frames)
ENTER_THRESHOLD = 5   # Trigger alert after N consecutive detections
EXIT_THRESHOLD = 8    # Stop alert after N consecutive non-detections

# Tracking Parameters
DEAD_ZONE = 25        # Error margin (pixels) to ignore small movements
RETURN_TIMEOUT = 3.0  # Reset to center after N seconds of no detection

# PID Parameters (Proportional only)
KP = 0.15             # Proportional gain
MAX_STEP = 5.0        # Max angle step per frame

# Servo Update Rate
SERVO_INTERVAL = 0.08  # Update every 80ms

# Center Angles (Adjust based on hardware assembly)
CENTER_PAN = 90
CENTER_TILT = 100

# Servo Direction Inversion
REVERSE_PAN = False
REVERSE_TILT = True
# =======================================

# Constants
NO_PERSON = 0
PERSON_DETECTED = 1

# Image Storage
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)


def save_screenshot(img):
    """Save screenshot with timestamp."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] {filepath}")
    return filepath


class Tracker:
    """Handles detection, servo control, and alert logic."""
    
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Current angles
        self.pan = float(CENTER_PAN)
        self.tilt = float(CENTER_TILT)
        
        self.last_servo_time = 0
        
        # Tracking state
        self.tracking = False
        self.last_seen_time = time.time()
        
        # Alert state
        self.alert_state = NO_PERSON
        self.detect_count = 0
        self.nodetect_count = 0
        
        # Hardware init
        self.pca = None
        self.servo_pan = None
        self.servo_tilt = None
        self._init_servos()
    
    def _init_servos(self):
        """Initialize PCA9685 and Servo objects."""
        try:
            i2c = board.I2C()
            self.pca = PCA9685(i2c)
            self.pca.frequency = 50  # 50Hz for standard servos
            
            # Channel 0: Pan, Channel 1: Tilt
            self.servo_pan = servo.Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500)
            self.servo_tilt = servo.Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500)
            
            # Reset to center
            self.servo_pan.angle = CENTER_PAN
            self.servo_tilt.angle = CENTER_TILT
            print(f"[SERVO] PCA9685 初始化成功 (Pan=Ch0@{CENTER_PAN}°, Tilt=Ch1@{CENTER_TILT}°)")
        except Exception as e:
            print(f"[ERROR] PCA9685 初始化失敗: {e}")
    
    def find_largest_person(self, results):
        """Return bounding box of the largest detected person."""
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
        """Calculate center coordinates of the bounding box."""
        if box is None:
            return None, None
        x1, y1, x2, y2 = box
        return (x1 + x2) / 2, (y1 + y2) / 2
    
    def update_tracking(self, target_x, target_y):
        """Calculate and update servo angles based on target position."""
        if target_x is None or target_y is None:
            return
        
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y
        
        # Apply dead zone
        if abs(error_x) < DEAD_ZONE: error_x = 0
        if abs(error_y) < DEAD_ZONE: error_y = 0
        
        # P Controller
        delta_pan = KP * error_x
        delta_tilt = KP * error_y
        
        # Reverse direction if needed
        if REVERSE_PAN: delta_pan = -delta_pan
        if REVERSE_TILT: delta_tilt = -delta_tilt
        
        # Limit movement step
        delta_pan = max(-MAX_STEP, min(MAX_STEP, delta_pan))
        delta_tilt = max(-MAX_STEP, min(MAX_STEP, delta_tilt))
        
        # Update and clamp angles
        self.pan = max(0, min(180, self.pan + delta_pan))
        self.tilt = max(0, min(180, self.tilt + delta_tilt))
    
    def move_servos(self):
        """Apply angles to servos (rate limited)."""
        now = time.time()
        if now - self.last_servo_time < SERVO_INTERVAL:
            return
        
        try:
            if self.servo_pan: self.servo_pan.angle = self.pan
            if self.servo_tilt: self.servo_tilt.angle = self.tilt
            self.last_servo_time = now
        except Exception as e:
            print(f"[ERROR] 馬達控制失敗: {e}")
    
    def reset_to_center(self):
        """Reset servos to the center position."""
        print(f"[TRACKING] 歸中 (Pan={CENTER_PAN}°, Tilt={CENTER_TILT}°)...")
        self.pan = float(CENTER_PAN)
        self.tilt = float(CENTER_TILT)
        try:
            if self.servo_pan: self.servo_pan.angle = CENTER_PAN
            if self.servo_tilt: self.servo_tilt.angle = CENTER_TILT
        except Exception:
            pass
        self.tracking = False
        self.last_seen_time = time.time()
    
    def update_alert(self, person_found, frame=None):
        """Handle alert logic and logging."""
        if person_found:
            self.detect_count += 1
            self.nodetect_count = 0
        else:
            self.nodetect_count += 1
            self.detect_count = 0
        
        # Trigger Alert
        if self.alert_state == NO_PERSON and self.detect_count >= ENTER_THRESHOLD:
            print("[ALERT] 偵測到入侵者! 警報開啟")
            self.alert_state = PERSON_DETECTED
            beep_pattern(duration=0.1, repeat=3, interval=0.05)
            buzzer_on()
            if frame is not None:
                img_path = save_screenshot(frame)
                send_email_with_latest_image()
                log_event("INTRUDER_DETECTED", image_path=img_path, detail="email_sent")
            else:
                log_event("INTRUDER_DETECTED", detail="no_frame_available")
        
        # Clear Alert
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] 入侵者離開，警報關閉")
            if frame is not None:
                img_path = save_screenshot(frame)
                log_event("INTRUDER_LEFT", image_path=img_path, detail="auto_screenshot_on_exit")
            else:
                log_event("INTRUDER_LEFT", detail="no_frame_available")
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_info(self, frame, box, tx, ty):
        """Overlay tracking info on the frame."""
        # Draw center crosshair
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                 (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                 (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        
        if box is not None and tx is not None:
            x1, y1, x2, y2 = box
            # Target bounding box
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 255), 3)
            # Target center
            cv2.circle(frame, (int(tx), int(ty)), 8, (0, 0, 255), -1)
            # Line to center
            cv2.line(frame, (self.center_x, self.center_y),
                     (int(tx), int(ty)), (255, 0, 255), 2)
            # Error text
            cv2.putText(frame, f"Error: ({self.center_x-tx:.0f}, {self.center_y-ty:.0f})", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)
        
        # Display Angles
        cv2.putText(frame, f"Pan: {self.pan:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Tilt: {self.tilt:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display Status
        status = "TRACKING" if self.tracking else "SEARCHING"
        color = (0, 255, 0) if self.tracking else (0, 165, 255)
        cv2.putText(frame, status, (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Display Alert Status
        alert = "ALARM!" if self.alert_state == PERSON_DETECTED else "STANDBY"
        acolor = (0, 0, 255) if self.alert_state == PERSON_DETECTED else (128, 128, 128)
        cv2.putText(frame, alert, (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, acolor, 2)
        
        return frame
    
    def cleanup(self):
        """Release hardware resources."""
        try:
            if self.servo_pan: self.servo_pan.angle = CENTER_PAN
            if self.servo_tilt: self.servo_tilt.angle = CENTER_TILT
            if self.pca: self.pca.deinit()
        except Exception:
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

    # Init System
    init_log()
    log_event("SYSTEM_START", detail=f"HEADLESS={HEADLESS}")
    
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
    log_event("SYSTEM_READY", detail="Camera+YOLO+Tracker initialized")
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    try:
        while True:
            frame = picam2.capture_array()
            frame_count += 1
            
            # YOLO Inference
            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            box = tracker.find_largest_person(results)
            person_found = box is not None
            
            # Alert Logic (Draw on annotated frame only if needed to save)
            if tracker.alert_state == NO_PERSON and tracker.detect_count >= ENTER_THRESHOLD - 1:
                annotated = results[0].plot()
                tracker.update_alert(person_found, annotated)
            elif tracker.alert_state == PERSON_DETECTED and tracker.nodetect_count >= EXIT_THRESHOLD - 1:
                annotated = results[0].plot()
                tracker.update_alert(person_found, annotated)
            else:
                tracker.update_alert(person_found, frame)
            
            # Tracking Logic
            if person_found:
                tx, ty = tracker.get_center(box)
                tracker.update_tracking(tx, ty)
                tracker.move_servos()
                tracker.tracking = True
                tracker.last_seen_time = time.time()
            else:
                tx, ty = None, None
                # Return to center on timeout
                if time.time() - tracker.last_seen_time > RETURN_TIMEOUT and tracker.tracking:
                    tracker.reset_to_center()
            
            # FPS Calculation
            if frame_count % 30 == 0:
                now = time.time()
                fps = 30 / (now - fps_time)
                fps_time = now
                print(f"[INFO] FPS: {fps:.1f} | Pan: {tracker.pan:.1f} | Tilt: {tracker.tilt:.1f} | Track: {tracker.tracking}")
            
            # Display
            if not HEADLESS:
                annotated = results[0].plot()
                display = tracker.draw_info(annotated, box, tx, ty)
                cv2.putText(display, f"FPS: {fps:.1f}",
                            (530, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2)
                cv2.imshow("Tracking", display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    tracker.reset_to_center()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
        log_event("SYSTEM_INTERRUPT", detail="KeyboardInterrupt")
    
    finally:
        print("[CLEANUP]...")
        buzzer_off()
        cleanup_buzzer()
        tracker.reset_to_center()
        tracker.cleanup()
        picam2.close()
        if not HEADLESS:
            cv2.destroyAllWindows()
        log_event("SYSTEM_STOP", detail="Normal cleanup finished")
        print("[EXIT] Done!")


if __name__ == "__main__":
    main()