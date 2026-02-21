from ultralytics import YOLO
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time
import cv2

# ---------------- CONFIG ----------------

BUTTON_PIN = 17
TRIG_PIN = 23
ECHO_PIN = 24
BUZZER_PIN = 18

DIST_THRESHOLD = 60  # cm
WHITELIST = ["chair", "dining table", "bench"]

# ----------------------------------------

# GPIO SETUP
GPIO.setmode(GPIO.BCM)

GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# YOLO MODEL
model = YOLO("yolov8n.pt")

# CAMERA SETUP
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())
picam2.start()

# ----------------------------------------

def buzzer_alert(duration=0.5):
    GPIO.output(BUZZER_PIN, True)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, False)

def get_distance():
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.05)

    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    elapsed = stop_time - start_time
    distance = (elapsed * 34300) / 2  # cm
    return distance

def capture_frame():
    frame = picam2.capture_array()
    frame = cv2.resize(frame, (640, 480))
    return frame

def detect_objects(frame):
    results = model(frame)
    detected = []

    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])

            if conf > 0.5:
                label = model.names[cls]
                detected.append(label)

    return detected

def is_dangerous(objects):
    if len(objects) == 0:
        return False

    for obj in objects:
        if obj not in WHITELIST:
            return True
    return False

# ----------------------------------------

print("System Ready. Press button to activate.")

system_on = False

try:
    while True:

        # BUTTON CHECK
        if GPIO.input(BUTTON_PIN) == 0:
            system_on = True
            print("System Activated")
            buzzer_alert(0.2)
            time.sleep(0.5)

        if system_on:

            distance = get_distance()
            print(f"Distance: {distance:.1f} cm")

            if distance < DIST_THRESHOLD:

                print("Object detected nearby. Running vision...")

                frame = capture_frame()
                objects = detect_objects(frame)

                print("Detected:", objects)

                if is_dangerous(objects):
                    print("DANGER OBJECT DETECTED")
                    for _ in range(3):
                        buzzer_alert(0.3)
                        time.sleep(0.2)
                else:
                    print("Object is safe")
                    buzzer_alert(0.1)

                time.sleep(2)  # Prevent spam scanning

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
