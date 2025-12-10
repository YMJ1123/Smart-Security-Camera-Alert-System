import sys
sys.path.append("/usr/lib/python3/dist-packages")  # 借系統的 picamera2

from ultralytics import YOLO
from picamera2 import Picamera2
import cv2
import libcamera
import time
from datetime import datetime
import os
from send_email import send_email_with_latest_image   # 寄信
from buzzer import init_buzzer, beep_pattern, cleanup_buzzer, buzzer_on, buzzer_off

# -------- 事件狀態常數 --------
NO_PERSON = 0
PERSON_DETECTED = 1

# -------- 參數可調 --------
ENTER_THRESHOLD = 5       # 5 幀連續有人 → 確認入侵
EXIT_THRESHOLD = 8        # 8 幀沒有偵測到人 → 離開
CAPTURE_INTERVAL = 10     # 事件中每 10 秒截圖一次

# 取得 detect.py 所在目錄，並在底下建 invade_image/
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")
os.makedirs(IMAGE_DIR, exist_ok=True)


def save_screenshot(img):
    """存圖片到 invade_image/，檔名用 timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"intruder_{timestamp}.png"
    filepath = os.path.join(IMAGE_DIR, filename)
    cv2.imwrite(filepath, img)
    print(f"[SAVE] Screenshot saved: {filepath}")
    return filepath


def main():
    # 初始化蜂鳴器
    init_buzzer()

    model = YOLO("yolo11n.pt")

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)},
        transform=libcamera.Transform(rotation=180),
    )
    picam2.configure(config)
    picam2.start()

    state = NO_PERSON
    detect_count = 0
    nodetect_count = 0
    last_capture_time = 0

    try:
        while True:
            frame = picam2.capture_array()

            results = model(frame, imgsz=320, classes=[0], conf=0.4, verbose=False)
            annotated = results[0].plot()
            big = cv2.resize(annotated, (1280, 960))

            # 有沒有人
            person_found = len(results[0].boxes) > 0

            if person_found:
                detect_count += 1
                nodetect_count = 0
            else:
                nodetect_count += 1
                detect_count = 0

            # ① 事件開始：從 NO_PERSON -> PERSON_DETECTED
            if state == NO_PERSON and detect_count >= ENTER_THRESHOLD:
                print("[EVENT] Intruder detected!")
                state = PERSON_DETECTED

                # 警報聲：先嗶幾下再改成長鳴（你也可以只用 buzzer_on）
                beep_pattern(duration=0.1, repeat=3, interval=0.05)
                buzzer_on()  # 之後持續叫到人離開

                save_screenshot(annotated)
                send_email_with_latest_image()  # 寄信

                last_capture_time = time.time()

            # ② 事件進行中：每 CAPTURE_INTERVAL 秒截一次
            if state == PERSON_DETECTED:
                now = time.time()
                if now - last_capture_time >= CAPTURE_INTERVAL:
                    save_screenshot(annotated)
                    last_capture_time = now

            # ③ 事件結束：從 PERSON_DETECTED -> NO_PERSON
            if state == PERSON_DETECTED and nodetect_count >= EXIT_THRESHOLD:
                print("[EVENT] Intruder left.")
                save_screenshot(annotated)

                # 關掉警報器
                buzzer_off()

                state = NO_PERSON

            cv2.imshow("YOLO11n person detect", big)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        picam2.close()
        cv2.destroyAllWindows()
        cleanup_buzzer()
        print("Clean exit.")


if __name__ == "__main__":
    main()
