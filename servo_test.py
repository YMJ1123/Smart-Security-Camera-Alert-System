from gpiozero import AngularServo
from time import sleep

SERVO1_PIN = 18  # GPIO18 - 伺服馬達 1
SERVO2_PIN = 23  # GPIO23 - 伺服馬達 2

# 如果轉動角度怪怪的，可以調 min_pulse_width / max_pulse_width
servo1 = AngularServo(
    SERVO1_PIN,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,  # 大約 0.5ms -> 0 度
    max_pulse_width=0.0025,  # 大約 2.5ms -> 180 度
)

servo2 = AngularServo(
    SERVO2_PIN,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025,
)

try:
    print("雙伺服馬達測試開始")
    print("Servo1 (GPIO18) 和 Servo2 (GPIO23) 會同時動作")
    print("按 Ctrl+C 結束\n")

    while True:
        for angle in (0, 90, 180, 90):
            print(f"兩個伺服馬達同時轉到 {angle}°")
            servo1.angle = angle
            servo2.angle = angle
            sleep(1.0)  # 等伺服轉完、穩定一下

except KeyboardInterrupt:
    print("\n測試結束")

finally:
    # 釋放腳位，避免抖動
    try:
        servo1.detach()
    except Exception:
        pass
    try:
        servo2.detach()
    except Exception:
        pass
