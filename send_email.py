import smtplib
import os
import glob
from datetime import datetime
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders

SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587
SENDER_EMAIL = "YOUR_SENDER_EMAIL" #please use your actual sender email here
SENDER_PASSWORD = "YOUR_APP_PASSWORD" # Use Google App Password
RECEIVER_EMAIL = "YOUR_RECEIVER_EMAIL" #please use your actual receiver email

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(BASE_DIR, "invade_image")

def send_email_with_latest_image():
    now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    subject = f"Warning!!! Intruder [{now_str}]"

    msg = MIMEMultipart()
    msg["From"] = SENDER_EMAIL
    msg["To"] = RECEIVER_EMAIL
    msg["Subject"] = subject

    body = (
        "Warning: Intruder detected.\n"
        "Please check the attached latest image.\n"
        f"Time: {now_str}\n\n"
        "此為智慧監控系統自動發送通知，請勿直接回覆此信。\n"
    )
    msg.attach(MIMEText(body, "plain"))

    # Find PNG files in the invade_image directory
    png_pattern = os.path.join(IMAGE_DIR, "*.png")
    png_files = glob.glob(png_pattern)

    if not png_files:
        print("No PNG images found.")
        return

    # Get the latest file
    latest_png = sorted(png_files)[-1]
    print("Latest image:", latest_png)

    with open(latest_png, "rb") as attachment:
        part = MIMEBase("application", "octet-stream")
        part.set_payload(attachment.read())

    encoders.encode_base64(part)
    filename_only = os.path.basename(latest_png)
    part.add_header("Content-Disposition", f"attachment; filename={filename_only}")

    msg.attach(part)

    try:
        server = smtplib.SMTP(SMTP_SERVER, SMTP_PORT)
        server.starttls()
        server.login(SENDER_EMAIL, SENDER_PASSWORD)
        server.send_message(msg)
        server.quit()
        print("Email sent successfully!")
    except Exception as e:
        print("Error sending email:", e)


if __name__ == "__main__":
    send_email_with_latest_image()
