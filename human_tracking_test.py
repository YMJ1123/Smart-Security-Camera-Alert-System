import sys
sys.path.append("/usr/lib/python3/dist-packages")  # Use system's picamera2

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time

# Buzzer module
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off

# Email module
from send_email import send_email_with_latest_image

# Event state constants
NO_PERSON = 0
PERSON_DETECTED = 1

# Alert thresholds (adjustable)
ENTER_THRESHOLD = 5   # 5 consecutive frames with person -> confirm intrusion
EXIT_THRESHOLD = 8    # 8 consecutive frames without person -> person left

# Screenshot directory
import os
from datetime import datetime
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)

# GPIO Servo control using gpiozero
# Note: Pi 5 uses software PWM which may have some jitter
# The code uses smoothing and threshold filtering to minimize this
from gpiozero import AngularServo

# Servo GPIO pins
SERVO_PAN_PIN = 18   # GPIO18 - Pan (horizontal)
SERVO_TILT_PIN = 23  # GPIO23 - Tilt (vertical)

# Tracking timeout (seconds) - return to center after this time without detection
RETURN_TO_CENTER_TIMEOUT = 3.0


def save_screenshot(img):
    """Save screenshot to invade_image/ with timestamp filename"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] Screenshot saved: {filepath}")
    return filepath


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
        
        # D term
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


class HumanTrackingTest:
    """Test version - displays tracking info WITH actual servo motor control"""
    def __init__(self, width=640, height=480):
        # Frame parameters
        self.width = width
        self.height = height
        self.center_x = width // 2
        self.center_y = height // 2
        
        # Servo angles (90 degrees is center position)
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
        self.servo_update_interval = 0.5  # Update every 0.5 seconds
        self.last_servo_update_time = 0
        
        # PID controller parameters - tuned for slow, fluid movement
        self.pid_pan = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        self.pid_tilt = PIDController(kp=0.03, ki=0.0001, kd=0.015, output_limit=1.5, integral_limit=15)
        
        # Dead zone
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
        
        # Store current target info for display
        self.current_target_area = 0
        self.current_target_center = (0, 0)
        self.delta_pan = 0
        self.delta_tilt = 0
        
        # Buzzer/Alert state
        self.alert_state = NO_PERSON
        self.detect_count = 0      # Consecutive frames with person detected
        self.nodetect_count = 0    # Consecutive frames without person
        
    def find_largest_person(self, results):
        """Find the largest person bounding box from YOLO results"""
        if len(results[0].boxes) == 0:
            return None, 0
        
        boxes = results[0].boxes
        max_area = 0
        largest_box = None
        
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            area = (x2 - x1) * (y2 - y1)
            if area > max_area:
                max_area = area
                largest_box = (x1, y1, x2, y2)
        
        return largest_box, max_area
    
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
            self.delta_pan = 0
            self.delta_tilt = 0
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
        self.delta_pan = self.pid_pan.compute(error_x)
        self.delta_tilt = self.pid_tilt.compute(error_y)
        
        # 4. Update raw angles
        self.pan_angle += self.delta_pan
        self.tilt_angle += self.delta_tilt
        
        # 5. Limit angle range (0~180)
        self.pan_angle = max(0, min(180, self.pan_angle))
        self.tilt_angle = max(0, min(180, self.tilt_angle))
        
        # 6. Apply low-pass filter for smoothing
        self.smoothed_pan = self.smoothed_pan + self.smooth_factor * (self.pan_angle - self.smoothed_pan)
        self.smoothed_tilt = self.smoothed_tilt + self.smooth_factor * (self.tilt_angle - self.smoothed_tilt)
        
        # 7. Set actual servo angles
        self.set_servo_angles(self.smoothed_pan, self.smoothed_tilt)
        
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
    
    def reset_to_center(self):
        """Reset servos to center position"""
        print("[SERVO] Resetting to center position...")
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
        self.delta_pan = 0
        self.delta_tilt = 0
    
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
            
            # Save screenshot and send email
            if frame is not None:
                save_screenshot(frame)
                send_email_with_latest_image()
        
        # State transition: PERSON_DETECTED -> NO_PERSON
        if self.alert_state == PERSON_DETECTED and self.nodetect_count >= EXIT_THRESHOLD:
            print("[ALERT] Intruder left. Buzzer OFF")
            
            # Save final screenshot before turning off
            if frame is not None:
                save_screenshot(frame)
            
            buzzer_off()
            self.alert_state = NO_PERSON
    
    def draw_tracking_info(self, frame, box, target_x, target_y, area):
        """Draw comprehensive tracking information on frame"""
        h, w = frame.shape[:2]
        
        # Draw crosshair at frame center
        cv2.line(frame, (self.center_x - 30, self.center_y), 
                (self.center_x + 30, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 30), 
                (self.center_x, self.center_y + 30), (0, 255, 0), 2)
        cv2.circle(frame, (self.center_x, self.center_y), 5, (0, 255, 0), -1)
        
        # Create info panel background (semi-transparent)
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (350, 220), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Title
        cv2.putText(frame, "=== TRACKING TEST MODE ===", 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Tracking status
        status = "TRACKING" if self.tracking_active else "SEARCHING"
        status_color = (0, 255, 0) if self.tracking_active else (0, 165, 255)
        cv2.putText(frame, f"Status: {status}", 
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # Display time until return to center
        if self.tracking_active and box is None:
            time_left = RETURN_TO_CENTER_TIMEOUT - (time.time() - self.last_detection_time)
            if time_left > 0:
                cv2.putText(frame, f"Return in: {time_left:.1f}s", 
                           (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        
        if box is not None and target_x is not None and target_y is not None:
            x1, y1, x2, y2 = box
            
            # Draw bounding box for largest person (cyan, thicker)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), 
                         (255, 255, 0), 3)
            
            # Draw "TRACKING TARGET" label above box
            cv2.putText(frame, "TRACKING TARGET", 
                       (int(x1), int(y1) - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Draw target center point (red dot)
            cv2.circle(frame, (int(target_x), int(target_y)), 10, (0, 0, 255), -1)
            cv2.circle(frame, (int(target_x), int(target_y)), 12, (255, 255, 255), 2)
            
            # Draw line from frame center to target
            cv2.line(frame, (self.center_x, self.center_y), 
                    (int(target_x), int(target_y)), (255, 0, 255), 2)
            
            # Calculate errors
            error_x = self.center_x - target_x
            error_y = self.center_y - target_y
            
            # Display target information
            cv2.putText(frame, f"Target Center: ({int(target_x)}, {int(target_y)})", 
                       (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(frame, f"Target Area: {int(area)} px^2", 
                       (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            cv2.putText(frame, f"Box Size: {int(x2-x1)}x{int(y2-y1)} px", 
                       (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            
            # Display error from center
            cv2.putText(frame, f"Error X: {error_x:+.1f} px", 
                       (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1)
            cv2.putText(frame, f"Error Y: {error_y:+.1f} px", 
                       (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1)
        else:
            cv2.putText(frame, "No target detected", 
                       (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (128, 128, 128), 1)
        
        # Display alert status
        alert_status = "ALARM ON" if self.alert_state == PERSON_DETECTED else "STANDBY"
        alert_color = (0, 0, 255) if self.alert_state == PERSON_DETECTED else (128, 128, 128)
        cv2.putText(frame, f"Buzzer: {alert_status}", 
                   (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.55, alert_color, 1)
        
        # Create servo info panel on the right side
        overlay2 = frame.copy()
        cv2.rectangle(overlay2, (w - 300, 5), (w - 5, 180), (0, 0, 0), -1)
        cv2.addWeighted(overlay2, 0.6, frame, 0.4, 0, frame)
        
        # Servo angle information
        cv2.putText(frame, "=== SERVO CONTROL ===", 
                   (w - 295, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        
        cv2.putText(frame, f"Pan Angle: {self.smoothed_pan:.1f} deg", 
                   (w - 295, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(frame, f"Tilt Angle: {self.smoothed_tilt:.1f} deg", 
                   (w - 295, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        
        cv2.putText(frame, f"Delta Pan: {self.delta_pan:+.2f} deg", 
                   (w - 295, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 1)
        cv2.putText(frame, f"Delta Tilt: {self.delta_tilt:+.2f} deg", 
                   (w - 295, 135), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 1)
        
        # Direction indicator
        direction = ""
        if abs(self.delta_pan) > 0.1 or abs(self.delta_tilt) > 0.1:
            if self.delta_pan > 0.1:
                direction += "LEFT "
            elif self.delta_pan < -0.1:
                direction += "RIGHT "
            if self.delta_tilt > 0.1:
                direction += "UP"
            elif self.delta_tilt < -0.1:
                direction += "DOWN"
        else:
            direction = "CENTERED"
        
        cv2.putText(frame, f"Moving: {direction}", 
                   (w - 295, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 100, 255), 1)
        
        # Draw servo position indicator (visual representation)
        self._draw_servo_indicator(frame, w - 150, h - 100)
        
        return frame
    
    def _draw_servo_indicator(self, frame, cx, cy):
        """Draw a visual servo position indicator"""
        # Background circle
        cv2.circle(frame, (cx, cy), 60, (50, 50, 50), -1)
        cv2.circle(frame, (cx, cy), 60, (100, 100, 100), 2)
        
        # Center point
        cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)
        
        # Draw crosshairs
        cv2.line(frame, (cx - 60, cy), (cx + 60, cy), (70, 70, 70), 1)
        cv2.line(frame, (cx, cy - 60), (cx, cy + 60), (70, 70, 70), 1)
        
        # Calculate indicator position based on smoothed angles
        # Map 0-180 degrees to -50 to +50 pixels
        ind_x = int((self.smoothed_pan - 90) / 90 * 50)
        ind_y = int((self.smoothed_tilt - 90) / 90 * 50)
        
        # Draw current position indicator
        cv2.circle(frame, (cx + ind_x, cy - ind_y), 8, (0, 0, 255), -1)
        cv2.circle(frame, (cx + ind_x, cy - ind_y), 10, (255, 255, 255), 2)
        
        # Label
        cv2.putText(frame, "Servo Position", 
                   (cx - 50, cy + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


def main():
    print("=" * 60)
    print("Human Tracking Camera - TEST MODE (With Servo Motors)")
    print("With buzzer alert system")
    print("=" * 60)
    print("This version controls REAL servo motors and displays:")
    print("  - Target area and coordinates")
    print("  - Servo angles (Pan=GPIO18, Tilt=GPIO23)")
    print("  - Movement direction")
    print("  - Buzzer alert (REAL buzzer connected)")
    print("=" * 60)
    print("Controls:")
    print("  q - Exit program")
    print("  r - Reset servos to center")
    print("=" * 60)
    
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
    
    # Initialize test tracking system
    print("[INIT] Initializing test tracking system...")
    tracker = HumanTrackingTest(width=640, height=480)
    
    print("[READY] Test system ready! Starting...")
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
            largest_box, area = tracker.find_largest_person(results)
            
            # Check if person is found
            person_found = largest_box is not None
            
            # Get annotated frame for screenshot
            annotated = results[0].plot()
            
            # Update buzzer alert state (with frame for screenshot)
            tracker.update_alert_state(person_found, annotated)
            
            if largest_box is not None:
                # Found person, calculate tracking
                target_x, target_y = tracker.get_box_center(largest_box)
                
                # Calculate servo angles
                pan, tilt = tracker.calculate_new_angles(target_x, target_y)
                
                # Update state
                tracker.tracking_active = True
                tracker.last_detection_time = time.time()
                
            else:
                # No person found
                target_x, target_y = None, None
                area = 0
                
                # Check if we should return to center (time-based)
                time_since_last_detection = time.time() - tracker.last_detection_time
                if time_since_last_detection > RETURN_TO_CENTER_TIMEOUT and tracker.tracking_active:
                    print(f"[TRACKING] No person detected for {RETURN_TO_CENTER_TIMEOUT:.1f}s, returning to center...")
                    tracker.reset_to_center()
            
            # Draw all tracking information
            display_frame = tracker.draw_tracking_info(annotated, largest_box, 
                                                       target_x, target_y, area)
            
            # Calculate FPS
            if frame_count % 30 == 0:
                current_time = time.time()
                fps = 30 / (current_time - fps_time)
                fps_time = current_time
            
            # Display FPS
            cv2.putText(display_frame, f"FPS: {fps:.1f}", 
                       (display_frame.shape[1] - 100, display_frame.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Resize for display
            big = cv2.resize(display_frame, (1280, 960))
            cv2.imshow("Human Tracking Test", big)
            
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
        print("[EXIT] Clean exit. Goodbye!")


if __name__ == "__main__":
    main()

