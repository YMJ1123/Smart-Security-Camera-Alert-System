import RPi.GPIO as GPIO
import time

# BCM
BUZZER_PIN = 24

def init_buzzer():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def buzzer_on():
    GPIO.output(BUZZER_PIN, GPIO.HIGH)

def buzzer_off():
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def beep_pattern(duration=0.2, repeat=5, interval=0.1):
    for _ in range(repeat):
        buzzer_on()
        time.sleep(duration)
        buzzer_off()
        time.sleep(interval)

def cleanup_buzzer():
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.cleanup()
