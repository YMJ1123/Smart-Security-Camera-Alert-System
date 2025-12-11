# log.py
import os
import csv
from datetime import datetime

# 這個檔案所在的目錄（final_project）
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 專門放 log 檔的資料夾：final_project/log
LOG_DIR = os.path.join(BASE_DIR, "log")
os.makedirs(LOG_DIR, exist_ok=True)  # 如果沒有就建立

# event_log.csv 放在 log/ 底下
LOG_FILE = os.path.join(LOG_DIR, "event_log.csv")


def init_log():
    """
    初始化 event_log.csv：
    - 若檔案不存在就建立，並寫入欄位名稱
    - 若已存在就不動它（保留舊紀錄）
    """
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "event", "image", "detail"])


def log_event(event: str, image_path: str = "", detail: str = ""):
    """
    寫一筆紀錄到 event_log.csv

    Parameters
    ----------
    event : str
        事件名稱，例如:
        - SYSTEM_START
        - SYSTEM_STOP
        - INTRUDER_DETECTED
        - INTRUDER_LEFT
        - SCREENSHOT
    image_path : str
        對應截圖的完整路徑或檔名（會自動只保留檔名）
    detail : str
        額外說明文字，例如:
        - "email_sent"
        - "pan=90, tilt=95"
    """
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    img_name = os.path.basename(image_path) if image_path else ""

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([ts, event, img_name, detail])

    # 終端機順便印一下（debug / demo 用）
    print(f"[LOG] {ts} | {event} | {img_name} | {detail}")
