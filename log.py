import os
import csv
from datetime import datetime

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "log")
os.makedirs(LOG_DIR, exist_ok=True)  
LOG_FILE = os.path.join(LOG_DIR, "event_log.csv")

def init_log():
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "event", "image", "detail"])


def log_event(event: str, image_path: str = "", detail: str = ""):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    img_name = os.path.basename(image_path) if image_path else ""

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([ts, event, img_name, detail])

    print(f"[LOG] {ts} | {event} | {img_name} | {detail}")
