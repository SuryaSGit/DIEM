from picamzero import Camera
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# ---------------- CONFIG ----------------

BUTTON_PIN = 17
TRIG_PIN = 23
ECHO_PIN = 24
BUZZER_PIN = 18

DIST_THRESHOLD = 60       # cm
MIN_CONTOUR_AREA = 6000   # tune this

# ---------------------------------------

# GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# CAMERA SETUP (picamzero)
cam = Camera()
cam.resolution = (640, 480)
time.sleep(2)

# Background subtractor
bg_subtractor = cv2.createBackgroundSubtractorMOG2(
    history=120,
    varThreshold=50,
    detectShadows=False
)

# ---------------------------------------

def buzzer_alert(duration=0.4):
    GPIO.output(BUZZER_PIN, True)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, False)

def get_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    while GPIO.input(ECHO_PIN) == 0:
        start = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        end = time.time()

    return ((end - start) * 34300) / 2

def capture_frame():
    frame = cam.capture_array()
    return frame

def detect_obstacle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)

    fgmask = bg_subtractor.apply(blur)
    _, thresh = cv2.threshold(fgmask, 200, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for cnt in contours:
        if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
            return True

    return False

# ---------------------------------------

print("System Ready. Press button to activate.")
system_on = False

try:
    while True:

        if GPIO.input(BUTTON_PIN) == 0:
            system_on = True
            print("System Activated")
            time.sleep(0.5)

        if system_on:
            distance = get_distance()
            print(f"Distance: {distance:.1f} cm")

            if distance < DIST_THRESHOLD:
                print("Object nearby — running vision")

                frame = capture_frame()
                danger = detect_obstacle(frame)

                if danger:
                    print("⚠️ DANGEROUS OBSTACLE")
                    for _ in range(3):
                        buzzer_alert()
                        time.sleep(0.2)
                else:
                    print("Area appears safe")

                time.sleep(1.5)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
    cam.close()