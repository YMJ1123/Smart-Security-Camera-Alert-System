============================================================
SMART SECURITY CAMERA ALERT SYSTEM
============================================================

This project implements an intelligent security surveillance system integrating YOLOv11 for human detection, automatic servo tracking, and real-time alert notifications.

============================================================
1. PROJECT OVERVIEW
============================================================

Powered by a Raspberry Pi, PiCamera2, and a PCA9685 servo controller, this system provides the following core features:

- YOLOv11 Detection: Real-time human recognition using the YOLOv11n model.
- Auto-Tracking: Pan/Tilt servo control to automatically follow detected targets.
- Alert System: Audible buzzer alarms combined with email notifications containing snapshots.
- Event Logging: Comprehensive event tracking recorded in CSV format.

============================================================
2. PROJECT STRUCTURE
============================================================

Smart-Security-Camera-Alert-System/
  |-- tracking_pca9685.py    # Main entry point (Tracking + Alert System)
  |-- buzzer.py              # Buzzer control module
  |-- send_email.py          # SMTP email notification module
  |-- log.py                 # Event logging module
  |-- readme.txt             # Documentation
  |-- yolo11n.pt             # YOLOv11 model file (Requires download)
  |-- invade_image/          # Storage for intruder snapshots (Auto-generated)
  |-- log/                   # Storage for logs (Auto-generated)
      |-- event_log.csv      # Event log file

============================================================
3. HARDWARE REQUIREMENTS
============================================================

- Raspberry Pi (4B or newer recommended)
- PiCamera2 Module
- PCA9685 16-Channel 12-bit PWM/Servo Driver
- 2x Servo Motors (for Pan and Tilt)
- Active Buzzer (Connected to GPIO 24)
- Internet Connection (Required for email alerts)

============================================================
4. INSTALLATION
============================================================

[Step 1] Environment Setup
It is recommended to run this project within a virtual environment.

    python3 -m venv venv
    source venv/bin/activate

[Step 2] Install Dependencies

    pip install --upgrade pip
    pip install ultralytics picamera2 opencv-python adafruit-circuitpython-pca9685 adafruit-circuitpython-motor RPi.GPIO

[Step 3] Download Model
Download the YOLOv11n model and place it in the project root directory.

    wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt

============================================================
5. CONFIGURATION
============================================================

(A) Email Settings
Update "send_email.py" with your SMTP credentials.

    SENDER_EMAIL = "your_email@gmail.com"
    SENDER_PASSWORD = "your_app_password"
    RECEIVER_EMAIL = "receiver_email@gmail.com"

    NOTE: For Gmail, you must use an App Password. 

(B) Tracking Parameters
Adjust the configuration section in "tracking_pca9685.py":

    ENTER_THRESHOLD = 5   (Trigger alert after N frames of detection)
    EXIT_THRESHOLD = 8    (Stop alert after N frames without detection)
    HEADLESS = False      (Set to True to disable GUI window for better performance)

============================================================
6. USAGE
============================================================

[Running the System]
Execute the main script within your virtual environment:

    python tracking_pca9685.py

[Controls]
    q      : Quit the application.
    r      : Manually reset camera to center position.
    Ctrl+C : Force stop/Interrupt.

[Workflow]
1. Initialization: System starts and centers the camera.
2. Monitoring: Continuous human detection using YOLO.
3. Detection Event:
   - Camera actively tracks the target.
   - Alert triggers after threshold is met (Buzzer activates).
   - Snapshot is taken and emailed to the recipient.
   - Event is logged to CSV.
4. Target Lost:
   - Alert deactivates.
   - Camera returns to the standby (center) position.

============================================================
7. DATA & LOGGING
============================================================

[Event Logs]
Located at: log/event_log.csv
Format: timestamp, event, image, detail

Example:
2025-12-17 14:30:22, PERSON_ENTER, intruder_2025.png, Alert triggered

[Snapshots]
Images are saved to the "invade_image/" directory.

============================================================
8. TROUBLESHOOTING
============================================================

- Camera Failure: Run "sudo raspi-config" > Interface Options > Camera > Enable.
- I2C Errors: Run "sudo raspi-config" > Interface Options > I2C > Enable.
- Email Failure: Check internet and verify Google App Password.