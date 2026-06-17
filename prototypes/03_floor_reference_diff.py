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
PIXEL_THRESHOLD = 8000  # Tune this — higher = less sensitive
# ---------------------------------------------

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.output(BUZZER_PIN, False)

# Camera Setup
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(2)  # Camera warm up

# ---------------------------------------------
def buzzer_alert(duration=0.3):
    GPIO.output(BUZZER_PIN, True)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, False)

# ---------------------------------------------
def get_distance():
    readings = []
    for _ in range(5):
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
        if elapsed <= 0 or elapsed > 0.04:
            readings.append(999)
        else:
            readings.append((elapsed * 34300) / 2)

        time.sleep(0.01)

    readings.sort()
    return sum(readings[1:-1]) / 3  # Average middle 3

# ---------------------------------------------
def capture_frame():
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame

# ---------------------------------------------
def capture_reference():
    print("Point camera at empty floor. Capturing reference in 3 seconds...")
    time.sleep(3)
    frame = capture_frame()
    cv2.imwrite("reference.jpg", frame)
    print("Reference image captured.")
    buzzer_alert(0.1)
    return frame

# ---------------------------------------------
def compare_to_reference(ref_frame):
    current = capture_frame()
    diff = cv2.absdiff(ref_frame, current)
    gray_diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_diff, 30, 255, cv2.THRESH_BINARY)
    changed_pixels = cv2.countNonZero(thresh)
    return changed_pixels

# ---------------------------------------------
print("Capturing reference image on startup...")
reference = capture_reference()
print("System Ready. Press button to activate.")

system_on = False

try:
    while True:
        # Button: short press = toggle system, long press = recapture reference
        if GPIO.input(BUTTON_PIN) == 0:
            press_start = time.time()
            while GPIO.input(BUTTON_PIN) == 0:
                time.sleep(0.01)
            press_duration = time.time() - press_start

            if press_duration > 2:
                # Long press — recapture reference
                print("Recapturing reference image...")
                reference = capture_reference()
            else:
                # Short press — toggle system
                system_on = not system_on
                print("System", "Activated" if system_on else "Deactivated")
                buzzer_alert(0.2)

            time.sleep(0.3)

        if system_on:
            distance = get_distance()
            print(f"Distance: {distance:.1f} cm")

            if distance < DIST_THRESHOLD:
                print("Object nearby → Running vision check")
                changed = compare_to_reference(reference)
                print(f"Changed pixels: {changed}")

                if changed > PIXEL_THRESHOLD:
                    print("Object detected → ALARM")
                    for _ in range(3):
                        buzzer_alert(0.3)
                        time.sleep(0.2)
                else:
                    print("No object detected → SAFE")
                    buzzer_alert(0.1)

                time.sleep(2)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
