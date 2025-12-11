from gpiozero import AngularServo
from time import sleep

# --- 設定 GPIO 腳位 ---
SERVO_PAN_PIN = 18   # 水平
SERVO_TILT_PIN = 23  # 垂直

# --- 初始化馬達 ---
print("正在初始化馬達...")

try:
    # 脈衝寬度設定維持與你原本的一致
    servo_pan = AngularServo(SERVO_PAN_PIN, min_angle=0, max_angle=180, 
                             min_pulse_width=0.0005, max_pulse_width=0.0025)
    
    servo_tilt = AngularServo(SERVO_TILT_PIN, min_angle=0, max_angle=180, 
                              min_pulse_width=0.0005, max_pulse_width=0.0025)
except Exception as e:
    print(f"初始化失敗: {e}")
    exit()

def smooth_move(servo, start_angle, end_angle, step_size=0.5, delay=0.01):
    """
    平滑移動函式
    :param servo: 馬達物件
    :param start_angle: 起始角度
    :param end_angle: 目標角度
    :param step_size: 每次移動的角度 (越小越平滑，例如 0.5 度)
    :param delay: 每次移動後的暫停時間 (秒，例如 0.01 秒)
    """
    print(f" -> 平滑移動: {start_angle} 到 {end_angle}")
    
    current_angle = float(start_angle)
    target_angle = float(end_angle)
    
    # 決定移動方向 (1 是增加角度，-1 是減少角度)
    direction = 1 if target_angle > current_angle else -1
    
    # 使用迴圈一點一點移動
    # 當 (方向是正 且 目前小於目標) 或 (方向是負 且 目前大於目標) 時繼續跑
    while (direction > 0 and current_angle < target_angle) or \
          (direction < 0 and current_angle > target_angle):
        
        current_angle += (step_size * direction)
        
        # 確保不要超過目標角度
        if direction > 0:
            current_angle = min(current_angle, target_angle)
        else:
            current_angle = max(current_angle, target_angle)
            
        # 設定角度
        servo.angle = current_angle
        # 短暫暫停，讓馬達跟上，也產生平滑視覺效果
        sleep(delay)

    # 確保最後準確停在目標點
    servo.angle = target_angle
    sleep(0.2) # 到達目的地後稍作停留

def test_single_servo(servo, name):
    print(f"\n--- 測試 {name} 馬達 ---")
    
    # 1. 先快速歸位到中間 (90度)
    print(f"[{name}] 歸位到 90度...")
    servo.angle = 90
    sleep(1)

    # 2. 從 90度 平滑轉到 0度
    print(f"[{name}] 步驟一: 往右轉到底 (90 -> 0)")
    smooth_move(servo, start_angle=90, end_angle=0, step_size=0.5, delay=0.01)
    sleep(0.5)

    # 3. 從 0度 平滑轉到 180度 (全景掃描)
    print(f"[{name}] 步驟二: 全景掃描 (0 -> 180)")
    smooth_move(servo, start_angle=0, end_angle=180, step_size=0.5, delay=0.01)
    sleep(0.5)

    # 4. 從 180度 平滑回到 90度
    print(f"[{name}] 步驟三: 回到中間 (180 -> 90)")
    smooth_move(servo, start_angle=180, end_angle=90, step_size=0.5, delay=0.01)
    sleep(0.5)

try:
    # 測試水平馬達
    test_single_servo(servo_pan, "水平 (Pan, GPIO 18)")
    
    # 測試垂直馬達
    test_single_servo(servo_tilt, "垂直 (Tilt, GPIO 23)")

    print("\n測試完成！馬達將被釋放。")

except KeyboardInterrupt:
    print("\n強制中斷測試")

finally:
    # 釋放馬達，停止出力
    # 這一步很重要，可以避免軟體PWM在靜止時產生的抖動噪音
    if 'servo_pan' in globals(): servo_pan.detach()
    if 'servo_tilt' in globals(): servo_tilt.detach()
    print("馬達已釋放 (Detached)")