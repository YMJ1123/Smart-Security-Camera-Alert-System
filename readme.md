# Smart Security Camera Alert System

An intelligent security monitoring system integrating YOLO human detection, automatic tracking, and alert notification.

## Overview

This system uses Raspberry Pi with PiCamera2 and PCA9685 servo controller to implement smart surveillance:
- **YOLO11 Human Detection**: Real-time person detection using YOLOv11n model
- **Auto-Tracking**: Pan/Tilt gimbal automatically tracks detected persons
- **Alert Notifications**: Buzzer alarm + Email notification with captured images
- **Event Logging**: CSV format logging of all detection events

## Project Structure

```
Smart-Security-Camera-Alert-System/
├── tracking_pca9685.py    # Main program (tracking + alert system)
├── buzzer.py              # Buzzer control module
├── send_email.py          # Email notification module
├── log.py                 # Event logging module
├── readme.md              # Documentation
├── yolo11n.pt             # YOLO11 model file (requires download)
├── invade_image/          # Intruder screenshots (auto-created)
└── log/                   # Event logs directory
    └── event_log.csv      # Event log file (auto-created)
```

## Hardware Requirements

- Raspberry Pi (4B or higher recommended)
- PiCamera2 module
- PCA9685 servo driver board
- 2x servo motors (Pan/Tilt)
- Buzzer (connected to GPIO 24)
- Network connection (for email notifications)

## Setup
### 1. Create Virtual Environment (Recommended)

```bash
python3 -m venv venv
source venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install --upgrade pip
pip install ultralytics picamera2 opencv-python
pip install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor
pip install RPi.GPIO
```

### 3. Download YOLO11 Model

```bash
wget https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt
```

Or manually download and place in project root directory.  

Model Download Link:  
https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt


## Configuration
### Email Notification

Edit `send_email.py` with your Gmail credentials:

```python
SENDER_EMAIL = "your_email@gmail.com"
SENDER_PASSWORD = "your_app_password"  # Use Google App Password
RECEIVER_EMAIL = "receiver_email@gmail.com"
```

> **Note**: Use [Google App Password](https://support.google.com/accounts/answer/185833), not your account password.

### Tracking Parameters

Adjust settings in `tracking_pca9685.py`:

```python
# Alert thresholds
ENTER_THRESHOLD = 5   # Consecutive frames to trigger alert
EXIT_THRESHOLD = 8    # Consecutive frames to stop alert

# Tracking parameters
DEAD_ZONE = 25        # Pixel threshold for movement
RETURN_TIMEOUT = 3.0  # Seconds before returning to center

# Servo center angles (adjust based on hardware)
CENTER_PAN = 90       # Horizontal center
CENTER_TILT = 100     # Vertical center

# Display mode
HEADLESS = False      # True = No window (performance), False = Debug window
```

## Usage
### Run the Program

```bash
python tracking_pca9685.py
```

### Controls

- **`q`**: Quit program
- **`r`**: Reset camera to center position
- **`Ctrl+C`**: Emergency stop

### Workflow

1. System starts, camera returns to center
2. Human detection begins
3. When person detected:
   - Camera automatically tracks target
   - Alert triggers after threshold reached
   - Buzzer sounds
   - Screenshot captured and emailed
   - Event logged to CSV
4. When target disappears:
   - Alert stops after threshold reached
   - Camera returns to center

## Module Functions
### `tracking_pca9685.py` - Main Program
- YOLO11 human detection
- PCA9685 servo motor control
- Auto-tracking algorithm (PID control)
- Alert state machine
- Integration of all modules

### `buzzer.py` - Buzzer Module
- Initialize buzzer (GPIO 24)
- `buzzer_on()` / `buzzer_off()`: Control buzzer
- `beep_pattern()`: Generate alert sound
- `cleanup_buzzer()`: Clean up GPIO

### `send_email.py` - Email Module
- Find latest screenshot
- Send alert email via Gmail SMTP
- Attach intruder image
- Timestamp notation

### `log.py` - Logging Module
- Initialize CSV log file
- `log_event(event, image_path, detail)`: Log events
- Format: timestamp | event | image | detail

## Event Log Example
Sample content from `log/event_log.csv`:

```csv
timestamp,event,image,detail
2025-12-17 14:30:15,SYSTEM_START,,System initialized
2025-12-17 14:30:22,PERSON_ENTER,intruder_20251217_143022.png,Alert triggered
2025-12-17 14:30:22,EMAIL_SENT,intruder_20251217_143022.png,Email sent successfully
2025-12-17 14:30:45,PERSON_LEAVE,,Alert stopped
```

## Screenshot Storage

All intruder screenshots are saved to `invade_image/` directory with filename format:
```
intruder_YYYYMMDD_HHMMSS.png
```

## Troubleshooting

### Camera Not Starting
```bash
sudo raspi-config
# Select Interface Options > Camera > Enable
```

### I2C Error (PCA9685)
```bash
sudo raspi-config
# Select Interface Options > I2C > Enable

# Check I2C devices
ls /dev/i2c*
```

### Email Send Failure
- Verify Google App Password is used
- Check network connection
- Ensure firewall allows SMTP (Port 587)


