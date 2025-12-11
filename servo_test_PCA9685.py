#!/usr/bin/env python3
"""
PCA9685 伺服馬達測試程式
測試 Pan (水平) 和 Tilt (垂直) 馬達是否正常運作
"""

import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import time

# 測試參數
TEST_DELAY = 1.0  # 每個動作之間的延遲（秒）

def main():
    print("=" * 60)
    print("PCA9685 伺服馬達測試程式")
    print("=" * 60)
    print("接線確認：")
    print("  Channel 0 (Pan) : 水平馬達 (左右旋轉)")
    print("  Channel 1 (Tilt): 垂直馬達 (上下旋轉)")
    print("=" * 60)
    print()
    
    # 初始化 PCA9685
    print("[1] 初始化 PCA9685...")
    try:
        i2c = board.I2C()
        pca = PCA9685(i2c)
        pca.frequency = 50  # 伺服馬達標準頻率 50Hz
        print("    ✓ PCA9685 初始化成功")
    except Exception as e:
        print(f"    ✗ 初始化失敗: {e}")
        print("    請檢查：")
        print("      1. PCA9685 是否正確連接到 I2C (SDA/SCL)")
        print("      2. 是否已安裝必要的函式庫")
        return
    
    # 初始化伺服馬達
    print("\n[2] 初始化伺服馬達...")
    try:
        # Channel 0 = Pan (水平/左右)
        servo_pan = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
        print("    ✓ Channel 0 (Pan) 初始化成功")
        
        # Channel 1 = Tilt (垂直/上下)
        servo_tilt = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
        print("    ✓ Channel 1 (Tilt) 初始化成功")
    except Exception as e:
        print(f"    ✗ 伺服馬達初始化失敗: {e}")
        return
    
    print("\n" + "=" * 60)
    print("開始測試...")
    print("=" * 60)
    
    try:
        # 步驟 1: 歸中
        print("\n[測試 1] 兩顆馬達歸中 (90度)")
        servo_pan.angle = 90
        servo_tilt.angle = 90
        print("    Pan: 90°, Tilt: 90°")
        time.sleep(TEST_DELAY * 2)
        
        # 步驟 2: 測試 Pan (水平馬達)
        print("\n[測試 2] 測試 Pan 馬達 (水平/左右)")
        print("    → 向左轉 (0度)...")
        servo_pan.angle = 0
        time.sleep(TEST_DELAY)
        
        print("    → 回到中間 (90度)...")
        servo_pan.angle = 90
        time.sleep(TEST_DELAY)
        
        print("    → 向右轉 (180度)...")
        servo_pan.angle = 180
        time.sleep(TEST_DELAY)
        
        print("    → 回到中間 (90度)...")
        servo_pan.angle = 90
        time.sleep(TEST_DELAY)
        
        # 步驟 3: 測試 Tilt (垂直馬達)
        print("\n[測試 3] 測試 Tilt 馬達 (垂直/上下)")
        print("    → 向下轉 (0度)...")
        servo_tilt.angle = 0
        time.sleep(TEST_DELAY)
        
        print("    → 回到中間 (90度)...")
        servo_tilt.angle = 90
        time.sleep(TEST_DELAY)
        
        print("    → 向上轉 (180度)...")
        servo_tilt.angle = 180
        time.sleep(TEST_DELAY)
        
        print("    → 回到中間 (90度)...")
        servo_tilt.angle = 90
        time.sleep(TEST_DELAY)
        
        # 步驟 4: 連續掃描測試
        print("\n[測試 4] Pan 連續掃描測試 (0° -> 180°)")
        for angle in range(0, 181, 15):
            print(f"    Pan: {angle}°")
            servo_pan.angle = angle
            time.sleep(0.3)
        
        print("    → 回到中間 (90度)...")
        servo_pan.angle = 90
        time.sleep(TEST_DELAY)
        
        print("\n[測試 5] Tilt 連續掃描測試 (0° -> 180°)")
        for angle in range(0, 181, 15):
            print(f"    Tilt: {angle}°")
            servo_tilt.angle = angle
            time.sleep(0.3)
        
        print("    → 回到中間 (90度)...")
        servo_tilt.angle = 90
        time.sleep(TEST_DELAY)
        
        # 步驟 5: 組合動作測試
        print("\n[測試 6] 組合動作測試")
        positions = [
            (90, 90, "中心"),
            (45, 45, "左上"),
            (135, 45, "右上"),
            (135, 135, "右下"),
            (45, 135, "左下"),
            (90, 90, "回到中心")
        ]
        
        for pan_angle, tilt_angle, description in positions:
            print(f"    → {description}: Pan={pan_angle}°, Tilt={tilt_angle}°")
            servo_pan.angle = pan_angle
            servo_tilt.angle = tilt_angle
            time.sleep(TEST_DELAY)
        
        # 步驟 6: 圓周運動測試
        print("\n[測試 7] 圓周運動測試")
        import math
        steps = 16
        radius = 45  # 偏移角度
        center = 90
        
        for i in range(steps + 1):
            angle_rad = (i / steps) * 2 * math.pi
            pan = center + radius * math.cos(angle_rad)
            tilt = center + radius * math.sin(angle_rad)
            print(f"    步驟 {i+1}/{steps+1}: Pan={pan:.0f}°, Tilt={tilt:.0f}°")
            servo_pan.angle = pan
            servo_tilt.angle = tilt
            time.sleep(0.3)
        
        # 最後歸中
        print("\n[完成] 測試結束，歸中...")
        servo_pan.angle = 90
        servo_tilt.angle = 90
        time.sleep(1)
        
        print("\n" + "=" * 60)
        print("✓ 所有測試完成!")
        print("=" * 60)
        print("\n如果馬達動作正常，表示接線正確！")
        print("如果某個馬達沒動作，請檢查：")
        print("  1. 該馬達是否接在正確的 Channel")
        print("  2. 電源線是否正確連接")
        print("  3. 電池/電源是否有電且電壓正確 (5-6V)")
        print()
        
    except KeyboardInterrupt:
        print("\n\n[中斷] 使用者按下 Ctrl+C")
    except Exception as e:
        print(f"\n[錯誤] 測試過程中發生錯誤: {e}")
    finally:
        print("\n[清理] 釋放資源...")
        try:
            servo_pan.angle = 90
            servo_tilt.angle = 90
            pca.deinit()
        except:
            pass
        print("[完成] 程式結束")

if __name__ == "__main__":
    main()

