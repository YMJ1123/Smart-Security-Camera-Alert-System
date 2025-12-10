# buzzer.py
import RPi.GPIO as GPIO
import time

# 你實際接的 BCM 腳位
BUZZER_PIN = 24


def init_buzzer():
    """初始化蜂鳴器 GPIO"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def buzzer_on():
    """長鳴：拉高蜂鳴器腳位"""
    GPIO.output(BUZZER_PIN, GPIO.HIGH)


def buzzer_off():
    """關閉蜂鳴器"""
    GPIO.output(BUZZER_PIN, GPIO.LOW)

def beep_pattern(duration=0.2, repeat=5, interval=0.1):
    """
    蜂鳴器叫聲模式：嗶嗶嗶...
    duration: 每次嗶多久 (秒)
    repeat:   嗶幾下
    interval: 兩次之間停多久
    """
    for _ in range(repeat):
        buzzer_on()
        time.sleep(duration)
        buzzer_off()
        time.sleep(interval)


def cleanup_buzzer():
    """關閉蜂鳴器並釋放 GPIO"""
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.cleanup()
