"""DIEM — Indoor Hazard Warning System.

A wearable Raspberry Pi device that warns visually impaired users about
obstacles in their path using a two-stage sensor pipeline:

    1. An HC-SR04 ultrasonic sensor continuously measures distance ahead.
       Vision only runs when something is closer than DIST_THRESHOLD, so the
       (relatively expensive) camera work isn't spent on empty hallways.

    2. When something is close, the Pi camera captures a frame and compares it
       against a reference image of the empty floor. Pixels that changed mark
       the obstacle; the obstacle's colour histogram is then matched against a
       user-registered "whitelist" object (e.g. a guide cane, a familiar bag).
       Whitelisted objects are treated as safe; anything else triggers a buzzer.

Controls (single GPIO button):
    short press  → toggle the system on/off
    long press   → recalibrate (recapture floor reference + whitelist object)

Run on the Raspberry Pi with:  python main.py
"""

import time

import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2


# ── Configuration ───────────────────────────────────────────────────────────
# BCM pin numbers.
BUTTON_PIN = 17
TRIG_PIN = 23
ECHO_PIN = 24
BUZZER_PIN = 18

DIST_THRESHOLD = 60        # cm — closer than this triggers a vision check
PIXEL_THRESHOLD = 8000     # changed pixels needed to count as a real obstacle
WHITELIST_TOLERANCE = 0.7  # 0–1 histogram-correlation score to call a match
FRAME_SIZE = (640, 480)

REFERENCE_IMG = "reference.jpg"
WHITELIST_IMG = "whitelist.jpg"


# ── Hardware setup ────────────────────────────────────────────────────────────
def setup_gpio() -> None:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.output(BUZZER_PIN, False)


def setup_camera() -> Picamera2:
    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"format": "RGB888", "size": FRAME_SIZE}
    )
    cam.configure(config)
    cam.start()
    time.sleep(2)  # sensor warm-up
    return cam


# ── Sensors and actuators ─────────────────────────────────────────────────────
def buzzer_alert(duration: float = 0.3) -> None:
    GPIO.output(BUZZER_PIN, True)
    time.sleep(duration)
    GPIO.output(BUZZER_PIN, False)


def get_distance() -> float:
    """Median-filtered HC-SR04 reading in cm.

    Takes five pulses, drops the high and low outlier, and averages the
    middle three. Each echo wait is timeout-guarded so a missed pulse can't
    hang the loop (the bug in the first prototype); failed reads report 999.
    """
    readings = []
    for _ in range(5):
        GPIO.output(TRIG_PIN, False)
        time.sleep(0.05)
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.00001)
        GPIO.output(TRIG_PIN, False)

        start_time = stop_time = time.time()

        timeout = time.time() + 0.05
        while GPIO.input(ECHO_PIN) == 0 and time.time() < timeout:
            start_time = time.time()

        timeout = time.time() + 0.05
        while GPIO.input(ECHO_PIN) == 1 and time.time() < timeout:
            stop_time = time.time()

        elapsed = stop_time - start_time
        if elapsed <= 0 or elapsed > 0.04:
            readings.append(999)          # out of range / missed echo
        else:
            readings.append((elapsed * 34300) / 2)  # speed of sound / 2

        time.sleep(0.01)

    readings.sort()
    return sum(readings[1:-1]) / 3        # mean of the middle three


# ── Vision ────────────────────────────────────────────────────────────────────
def capture_frame(cam: Picamera2) -> np.ndarray:
    frame = cam.capture_array()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def get_object_mask(ref_frame: np.ndarray, current_frame: np.ndarray) -> np.ndarray:
    """Binary mask of pixels that differ from the empty-floor reference."""
    diff = cv2.absdiff(ref_frame, current_frame)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 30, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    return cv2.dilate(mask, kernel, iterations=2)


def get_histogram(frame: np.ndarray, mask: np.ndarray | None = None) -> np.ndarray:
    """Normalised hue/saturation histogram, optionally over a masked region."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], mask, [50, 60], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist


def capture_reference(cam: Picamera2) -> np.ndarray:
    print("Point camera at empty floor. Capturing reference in 3 seconds...")
    time.sleep(3)
    frame = capture_frame(cam)
    cv2.imwrite(REFERENCE_IMG, frame)
    print("Reference floor image captured.")
    buzzer_alert(0.1)
    return frame


def capture_whitelist(cam: Picamera2, reference: np.ndarray) -> np.ndarray:
    """Register a known-safe object by its colour histogram."""
    print("Place the whitelist object in front of the camera.")
    print("Capturing whitelist object in 3 seconds...")
    time.sleep(3)
    frame = capture_frame(cam)
    mask = get_object_mask(reference, frame)

    if cv2.countNonZero(mask) < 500:
        print("WARNING: object not distinct from floor; using full-frame histogram.")
        hist = get_histogram(frame)
    else:
        print(f"Object mask area: {cv2.countNonZero(mask)} pixels")
        hist = get_histogram(frame, mask)

    cv2.imwrite(WHITELIST_IMG, frame)
    print("Whitelist object captured.")
    buzzer_alert(0.1)
    buzzer_alert(0.1)
    return hist


def check_object(
    cam: Picamera2, reference: np.ndarray, whitelist_hist: np.ndarray
) -> str:
    """Classify what's ahead as 'no_object', 'whitelisted', or 'alarm'."""
    current = capture_frame(cam)

    mask = get_object_mask(reference, current)
    changed_pixels = cv2.countNonZero(mask)
    print(f"Changed pixels: {changed_pixels}")
    if changed_pixels < PIXEL_THRESHOLD:
        return "no_object"

    hist = get_histogram(current, mask)
    score = cv2.compareHist(whitelist_hist, hist, cv2.HISTCMP_CORREL)
    print(f"Whitelist match score: {score:.2f}")
    return "whitelisted" if score >= WHITELIST_TOLERANCE else "alarm"


# ── Main loop ─────────────────────────────────────────────────────────────────
def main() -> None:
    setup_gpio()
    cam = setup_camera()

    print("=== STARTUP ===")
    reference = capture_reference(cam)
    whitelist_hist = capture_whitelist(cam, reference)

    print("System Ready. Press button to activate.")
    print("Short press = toggle on/off | Long press (2s) = recalibrate")

    system_on = False
    try:
        while True:
            if GPIO.input(BUTTON_PIN) == 0:
                press_start = time.time()
                while GPIO.input(BUTTON_PIN) == 0:
                    time.sleep(0.01)
                press_duration = time.time() - press_start

                if press_duration > 2:
                    print("Recalibrating...")
                    reference = capture_reference(cam)
                    whitelist_hist = capture_whitelist(cam, reference)
                    print("Recalibration done.")
                else:
                    system_on = not system_on
                    print("System", "Activated" if system_on else "Deactivated")
                    buzzer_alert(0.2)

                time.sleep(0.3)

            if system_on:
                distance = get_distance()
                print(f"Distance: {distance:.1f} cm")

                if distance < DIST_THRESHOLD:
                    print("Object nearby → running vision check")
                    result = check_object(cam, reference, whitelist_hist)

                    if result == "no_object":
                        print("No significant object → SAFE")
                    elif result == "whitelisted":
                        print("Whitelisted object → SAFE")
                        buzzer_alert(0.1)
                    else:  # alarm
                        print("Unknown obstacle → ALARM")
                        for _ in range(3):
                            buzzer_alert(0.3)
                            time.sleep(0.2)

                    time.sleep(2)  # debounce repeated scans

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
