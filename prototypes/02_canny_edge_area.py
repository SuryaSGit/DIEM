import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from picamera2 import Picamera2

# ---------------- GPIO CONFIG ----------------

BUTTON_PIN = 17
TRIG_PIN = 23
ECHO_PIN = 24
BUZZER_PIN = 18

DIST_THRESHOLD = 60   # cm

# Contour area thresholds (TUNE THESE LIVE)
SAFE_AREA = 25000     # Large object = safe (chair/table)
MIN_OBJECT_AREA = 3000  # Minimum to count as object

# ---------------------------------------------

GPIO.setmode(GPIO.BCM)

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Camera Setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# ---------------------------------------------

def buzzer_alert(duration=0.3):
    GPIO.output(BUZZER_PIN, True)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, False)

# ---------------------------------------------

def get_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    stop_time = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(ECHO_PIN) == 0 and time.time() < timeout:
        start_time = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(ECHO_PIN) == 1 and time.time() < timeout:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2
    return distance

# ---------------------------------------------

def capture_frame():
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    return frame

# ---------------------------------------------

def detect_object_area(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > largest_area:
            largest_area = area

    return largest_area

# ---------------------------------------------

print("System Ready. Press button to activate.")

system_on = False

try:
    while True:

        # Button Press = Activate
        if GPIO.input(BUTTON_PIN) == 0:
            system_on = True
            print("System Activated")
            buzzer_alert(0.2)
            time.sleep(0.5)

        if system_on:

            distance = get_distance()
            print(f"Distance: {distance:.1f} cm")

            if distance < DIST_THRESHOLD:

                print("Object nearby → Running vision")

                frame = capture_frame()
                area = detect_object_area(frame)

                print(f"Detected Area: {area}")

                if area < MIN_OBJECT_AREA:
                    print("No real object detected")

                elif area > SAFE_AREA:
                    print("Large object → SAFE")
                    buzzer_alert(0.1)

                else:
                    print("Medium object → ALARM")
                    for _ in range(3):
                        buzzer_alert(0.3)
                        time.sleep(0.2)

                time.sleep(2)  # Prevent spam scanning

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
