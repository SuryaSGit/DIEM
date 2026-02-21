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
PIXEL_THRESHOLD = 8000  # Tune this
WHITELIST_TOLERANCE = 0.7  # 0-1, higher = stricter matching
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
    print("Reference floor image captured.")
    buzzer_alert(0.1)
    return frame

# ---------------------------------------------
def get_object_mask(ref_frame, current_frame):
    diff = cv2.absdiff(ref_frame, current_frame)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
    # Dilate mask slightly to capture full object
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    return mask

# ---------------------------------------------
def get_histogram(frame, mask=None):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], mask, [50, 60], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist

# ---------------------------------------------
def capture_whitelist():
    print("Place duct tape roll in front of camera.")
    print("Capturing whitelist object in 3 seconds...")
    time.sleep(3)
    frame = capture_frame()
    mask = get_object_mask(reference, frame)

    # Check if we actually see something different from floor
    if cv2.countNonZero(mask) < 500:
        print("WARNING: Could not detect object against floor, capturing full frame histogram instead.")
        hist = get_histogram(frame)
    else:
        print(f"Object mask area: {cv2.countNonZero(mask)} pixels")
        hist = get_histogram(frame, mask)

    cv2.imwrite("whitelist.jpg", frame)
    print("Whitelist object captured.")
    buzzer_alert(0.1)
    buzzer_alert(0.1)
    return hist

# ---------------------------------------------
def check_object(ref_frame, whitelist_hist):
    current = capture_frame()
    
    # Step 1: Check how many pixels changed vs floor reference
    mask = get_object_mask(ref_frame, current)
    changed_pixels = cv2.countNonZero(mask)
    print(f"Changed pixels: {changed_pixels}")

    if changed_pixels < PIXEL_THRESHOLD:
        return "no_object"

    # Step 2: Compare color histogram of changed region to whitelist
    hist = get_histogram(current, mask)
    score = cv2.compareHist(whitelist_hist, hist, cv2.HISTCMP_CORREL)
    print(f"Whitelist match score: {score:.2f}")

    if score >= WHITELIST_TOLERANCE:
        return "whitelisted"
    else:
        return "alarm"

# ---------------------------------------------
# Startup sequence
print("=== STARTUP ===")
print("Step 1: Capture floor reference")
reference = capture_reference()

print("Step 2: Capture whitelist object")
whitelist_hist = capture_whitelist()

print("System Ready. Press button to activate.")
print("Short press = toggle on/off | Long press (2s) = recapture reference & whitelist")

system_on = False

try:
    while True:
        # Button handling
        if GPIO.input(BUTTON_PIN) == 0:
            press_start = time.time()
            while GPIO.input(BUTTON_PIN) == 0:
                time.sleep(0.01)
            press_duration = time.time() - press_start

            if press_duration > 2:
                # Long press — redo full calibration
                print("Recalibrating...")
                reference = capture_reference()
                whitelist_hist = capture_whitelist()
                print("Recalibration done.")
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
                result = check_object(reference, whitelist_hist)

                if result == "no_object":
                    print("No significant object detected → SAFE")

                elif result == "whitelisted":
                    print("Whitelisted object detected (duct tape) → SAFE")
                    buzzer_alert(0.1)

                elif result == "alarm":
                    print("Unknown object detected → ALARM")
                    for _ in range(3):
                        buzzer_alert(0.3)
                        time.sleep(0.2)

                time.sleep(2)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
