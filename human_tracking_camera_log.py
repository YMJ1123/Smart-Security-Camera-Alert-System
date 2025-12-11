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

# Logging 
from log import init_log, log_event

# Event state constants
NO_PERSON = 0
PERSON_DETECTED = 1

# Alert thresholds (adjustable)
ENTER_THRESHOLD = 5   # 5 consecutive frames with person -> confirm intrusion
EXIT_THRESHOLD = 8    # 8 consecutive frames without person -> person left

# Screenshot directory
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)


def save_screenshot(img, tag="event"):
    """Save screenshot to invade_image/ with timestamp filename and write log"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}_{tag}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] Screenshot saved: {filepath}")

    # 寫一筆 SCREENSHOT log
    log_event("SCREENSHOT", filepath, detail=f"tag={tag}")
    return filepath


# GPIO Servo control using gpiozero
# Note: Pi 5 uses software PWM which may have some jitter
# The code uses smoothing and threshold filtering to minimize this
from gpiozero import AngularServo

# Servo GPIO pins
SERVO_PAN_PIN = 18   # GPIO18 - Pan (horizontal)
SERVO_TILT_PIN = 23  # GPIO23 - Tilt (vertical)

# Tracking timeout (seconds) - return to center after this time without detection
RETURN_TO_CENTER_TIMEOUT = 3.0


class PIDController:
    """PID controller for calculating servo correction values with anti-windup"""
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
        
        # I term with anti-windup
        self.integral += error * dt
        # Limit integral to prevent windup
        self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i_term = self.ki * self.integral
        
        # D term with filtering (low-pass filter on derivative)
        raw_derivative = (error - self.prev_error) / dt
        d_term = self.kd * raw_derivative

        output = p_term + i_term + d_term

        # Update state
        self.prev_error = error
        self.last_time = current_time

        # Limit output range
        if self.output_limit:
            output = max(min(output, self.output_limit), -self.output_limit)
        
        self.prev_output = output
        return output

    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.prev_output = 0.0


class HumanTrackingCamera:
    """Camera system combining human detection with servo tracking"""
    def __init__(self, width=640, height=480):
        # Frame parameters
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Initialize servo angles (90 degrees is center position)
        self.pan_angle = 90.0   # Horizontal (left-right)
        self.tilt_angle = 90.0  # Vertical (up-down)
        
        # Smoothed angles (for reducing jitter)
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.smooth_factor = 0.12  # Smoother continuous movement
        
        # Last sent angles (only update servo when change is significant)
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        self.min_angle_change = 0.5  # Balance between smoothness and responsiveness
        
        # Update rate control - only update servo every N seconds
        self.servo_update_interval = 3  # Update every 3 seconds
        self.last_servo_update_time = 0
        
        # PID controller parameters - tuned for slow, fluid movement
        self.pid_pan = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        self.pid_tilt = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        
        # Dead zone: no action if error is smaller than this value
        self.dead_zone = 35
        
        # GPIO Servo control using gpiozero
        self.servo_pan = None
        self.servo_tilt = None
        try:
            self.servo_pan = AngularServo(
                SERVO_PAN_PIN,
                min_angle=0,
                max_angle=180,
                min_pulse_width=0.0005,  # 0.5ms -> 0 degrees
                max_pulse_width=0.0025,  # 2.5ms -> 180 degrees
            )
            self.servo_tilt = AngularServo(
                SERVO_TILT_PIN,
                min_angle=0,
                max_angle=180,
                min_pulse_width=0.0005,
                max_pulse_width=0.0025,
            )
            # Initialize to center position
            self.servo_pan.angle = self.pan_angle
            self.servo_tilt.angle = self.tilt_angle
            print(f"[SERVO] GPIO Servos initialized. Pan=GPIO{SERVO_PAN_PIN}, Tilt=GPIO{SERVO_TILT_PIN}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize GPIO servos: {e}")
            self.servo_pan = None
            self.servo_tilt = None
        
        # Tracking state
        self.tracking_active = False
        self.last_detection_time = time.time()  # Time-based return to center
        
        # Buzzer/Alert state
        self.alert_state = NO_PERSON
        self.detect_count = 0      # Consecutive frames with person detected
        self.nodetect_count = 0    # Consecutive frames without person
        
    def find_largest_person(self, results):
        """Find the largest person bounding box from YOLO results"""
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
        """Calculate the center point of bounding box"""
        if box is None:
            return None, None
        x1, y1, x2, y2 = box
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        return center_x, center_y
    
    def calculate_new_angles(self, target_x, target_y):
        """Calculate new servo angles based on target position with smoothing"""
        if target_x is None or target_y is None:
            return self.smoothed_pan, self.smoothed_tilt
        
        # 1. Calculate error (distance of target from frame center)
        error_x = self.center_x - target_x
        error_y = self.center_y - target_y
        
        # 2. Apply dead zone
        if abs(error_x) < self.dead_zone:
            error_x = 0
        if abs(error_y) < self.dead_zone:
            error_y = 0
        
        # 3. PID calculates correction amount
        delta_pan = self.pid_pan.compute(error_x)
        delta_tilt = self.pid_tilt.compute(error_y)
        
        # 4. Update raw angles
        # Pan: target on left (error_x > 0) needs to turn left (angle increases)
        self.pan_angle += delta_pan
        # Tilt: target on top (error_y > 0) needs to turn up (angle increases)
        self.tilt_angle += delta_tilt
        
        # 5. Limit angle range (0~180)
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))
        
        # 6. Apply low-pass filter for smoothing
        self.smoothed_pan = self.smoothed_pan + self.smooth_factor * (self.pan_angle - self.smoothed_pan)
        self.smoothed_tilt = self.smoothed_tilt + self.smooth_factor * (self.tilt_angle - self.smoothed_tilt)
        
        return self.smoothed_pan, self.smoothed_tilt
    
    def set_servo_angles(self, pan, tilt):
        """Set servo angles with rate limiting (updates every servo_update_interval seconds)"""
        current_time = time.time()
        
        # Check if enough time has passed since last update
        if current_time - self.last_servo_update_time < self.servo_update_interval:
            return  # Skip this update, not enough time has passed
        
        try:
            # Only update pan if change is significant
            if self.servo_pan is not None:
                if abs(pan - self.last_sent_pan) >= self.min_angle_change:
                    self.servo_pan.angle = pan
                    self.last_sent_pan = pan
            
            # Only update tilt if change is significant
            if self.servo_tilt is not None:
                if abs(tilt - self.last_sent_tilt) >= self.min_angle_change:
                    self.servo_tilt.angle = tilt
                    self.last_sent_tilt = tilt
            
            self.last_servo_update_time = current_time
        except Exception as e:
            print(f"[ERROR] Failed to set servo angles: {e}")
    
    def reset_to_center(self, smooth=False):
        """Reset servos to center position"""
        print("[TRACKING] Resetting to center position...")
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        self.smoothed_pan = 90.0
        self.smoothed_tilt = 90.0
        self.last_sent_pan = 90.0
        self.last_sent_tilt = 90.0
        
        # Force update servos to center
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

        # 寫一筆 RESET_CENTER log
        log_event("RESET_CENTER", detail="Servos reset to pan=90, tilt=90")

    
    def update_alert_state(self, person_found, frame=None):
        """Update buzzer alert state based on detection"""
        if person_found:
            self.detect_count += 1
            self.nodetect_count = 0
        else:
            self.nodetect_count += 1
            self.detect_count = 0
        
        # State transition: NO_PERSON -> PERSON_DETECTED
        if self.alert_state == NO_PERSON and self.detect_count >= ENTER_THRESHOLD:
            print("[ALERT] Intruder detected! Buzzer ON")
            self.alert_state = PERSON_DETECTED
            
            # Sound pattern then continuous alarm
            beep_pattern(duration=0.1, repeat=3, interval=0.05)
            buzzer_on()
            
            # Save screenshot + log + email
            if frame is not None:
                img_path = save_screenshot(frame, tag="enter")
                log_event(
                    "INTRUDER_DETECTED",
                    img_path,
                    detail=f"enter pan={self.pan_angle:.1f}, tilt={self.tilt_angle:.1f}",
                )
                send_email_with_latest_image()
            else:
                log_event("INTRUDER_DETECTED", detail="enter (no frame)")
        
        # State transition: PERSON_DETECTED -> NO_PERSON
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] Intruder left. Buzzer OFF")
            
            # Save final screenshot + log
            if frame is not None:
                img_path = save_screenshot(frame, tag="exit")
                log_event(
                    "INTRUDER_LEFT",
                    img_path,
                    detail=f"exit pan={self.pan_angle:.1f}, tilt={self.tilt_angle:.1f}",
                )
            else:
                log_event("INTRUDER_LEFT", detail="exit (no frame)")
            
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_tracking_info(self, frame, box, target_x, target_y):
        """Draw tracking information on frame"""
        # Draw crosshair at frame center
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        
        # If there's a target, draw bounding box and connection line
        if box is not None and target_x is not None and target_y is not None:
            x1, y1, x2, y2 = box
            # Draw bounding box (thicker, different color)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                         (0, 255, 255), 3)
            
            # Draw target center point
            cv2.circle(frame, (int(target_x), int(target_y)), 8, (0, 0, 255), -1)
            
            # Draw line from frame center to target
            cv2.line(frame, (self.center_x, self.center_y), 
                    (int(target_x), int(target_y)), (255, 0, 255), 2)
            
            # Display error information
            error_x = self.center_x - target_x
            error_y = self.center_y - target_y
            cv2.putText(frame, f"Error: X={error_x:.1f}, Y={error_y:.1f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display servo angles
        cv2.putText(frame, f"Pan: {self.pan_angle:.1f} deg", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Tilt: {self.tilt_angle:.1f} deg", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display tracking status
        status = "TRACKING" if self.tracking_active else "SEARCHING"
        color = (0, 255, 0) if self.tracking_active else (0, 165, 255)
        cv2.putText(frame, f"Status: {status}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Display time until return to center
        if self.tracking_active and box is None:
            time_left = RETURN_TO_CENTER_TIMEOUT - (time.time() - self.last_detection_time)
            if time_left > 0:
                cv2.putText(frame, f"Return in: {time_left:.1f}s", 
                           (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        
        # Display alert status
        alert_status = "ALARM ON" if self.alert_state == PERSON_DETECTED else "STANDBY"
        alert_color = (0, 0, 255) if self.alert_state == PERSON_DETECTED else (128, 128, 128)
        cv2.putText(frame, f"Alert: {alert_status}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, alert_color, 2)
        
        return frame


def main():
    print("=" * 50)
    print("Human Tracking Camera System")
    print("Uses YOLO to detect humans and PCA9685 to control servo tracking")
    print("With buzzer alert system")
    print("=" * 50)
    print("Controls:")
    print("  q - Exit program")
    print("  r - Reset servos to center position")
    print("=" * 50)
    
    # Initialize buzzer
    print("[INIT] Initializing buzzer...")
    init_buzzer()
    
    # Initialize YOLO model
    print("[INIT] Loading YOLO model...")
    model = YOLO("yolo11n.pt")
    
    # Initialize camera
    print("[INIT] Initializing camera...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=libcamera.Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Let camera stabilize

    # Initialize log
    init_log()
    log_event("SYSTEM_START", detail="Program started")

    # Initialize tracking system
    print("[INIT] Initializing tracking system...")
    tracker = HumanTrackingCamera(width=640, height=480)
    
    print("[READY] System ready! Starting tracking...")
    print()
    
    frame_count = 0
    fps_time = time.time()
    fps = 0
    
    try:
        while True:
            # Capture frame
            frame = picam2.capture_array()
            frame_count += 1
            
            # YOLO detect humans (class 0 = person)
            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            
            # Find the largest human target
            largest_box = tracker.find_largest_person(results)
            
            # Check if person is found
            person_found = largest_box is not None
            
            # Get annotated frame for screenshot
            annotated = results[0].plot()
            
            # Update buzzer alert state (with frame for screenshot)
            tracker.update_alert_state(person_found, annotated)
            
            if largest_box is not None:
                # Found person, start tracking
                target_x, target_y = tracker.get_box_center(largest_box)
                
                # Calculate new servo angles
                pan, tilt = tracker.calculate_new_angles(target_x, target_y)
                
                # Set servos
                tracker.set_servo_angles(pan, tilt)
                
                # Update state
                tracker.tracking_active = True
                tracker.last_detection_time = time.time()
                
            else:
                # No person found
                target_x, target_y = None, None
                
                # Check if we should return to center (time-based)
                time_since_last_detection = time.time() - tracker.last_detection_time
                if time_since_last_detection > RETURN_TO_CENTER_TIMEOUT and tracker.tracking_active:
                    print(f"[TRACKING] No person detected for {RETURN_TO_CENTER_TIMEOUT:.1f}s, returning to center...")
                    tracker.reset_to_center()
            
            # Draw tracking information
            display_frame = tracker.draw_tracking_info(annotated, largest_box, 
                                                       target_x, target_y)
            
            # Calculate FPS
            if frame_count % 30 == 0:
                current_time = time.time()
                fps = 30 / (current_time - fps_time)
                fps_time = current_time
            
            # Display FPS
            cv2.putText(display_frame, f"FPS: {fps:.1f}", 
                       (display_frame.shape[1] - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Resize for display
            big = cv2.resize(display_frame, (1280, 960))
            cv2.imshow("Human Tracking Camera", big)
            
            # Keyboard control
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[EXIT] User pressed 'q', exiting...")
                break
            elif key == ord('r'):
                tracker.reset_to_center()
    
    except KeyboardInterrupt:
        print("\n[EXIT] Keyboard interrupt detected...")
    
    finally:
        print("[CLEANUP] Cleaning up...")
        # Turn off buzzer and cleanup
        buzzer_off()
        cleanup_buzzer()
        # Reset servos to center and cleanup
        tracker.reset_to_center()
        # Detach servos to avoid jitter
        try:
            if tracker.servo_pan is not None:
                tracker.servo_pan.detach()
            if tracker.servo_tilt is not None:
                tracker.servo_tilt.detach()
        except Exception:
            pass
        picam2.close()
        cv2.destroyAllWindows()
        log_event("SYSTEM_STOP", detail="Program terminated")1
        print("[EXIT] Clean exit. Goodbye!")


if __name__ == "__main__":
    main()

