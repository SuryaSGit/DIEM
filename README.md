# DIEM — Indoor Hazard Warning System

> A wearable Raspberry Pi device that warns visually impaired users about
> obstacles in their path, in real time, using audio alerts.


🏆 **Placed 4th out of 100+ teams** at PULSE Hackathon

## What it does

DIEM combines an ultrasonic distance sensor with a camera so a visually
impaired user gets a buzzer warning *only* when an unfamiliar obstacle is
actually in their path — not for every wall, doorway, or piece of furniture
they already expect.

The key idea is a **two-stage pipeline** that keeps the device responsive and
cuts false alarms:

```
ultrasonic sensor  →  is something within 60 cm?
                          │ no  → keep scanning (camera idle)
                          │ yes → capture frame
                                    →  diff against empty-floor reference
                                    →  did enough pixels change?  (obstacle present?)
                                          │ no  → SAFE
                                          │ yes → match obstacle's colour histogram
                                                    against the registered "whitelist" object
                                                      │ match    → SAFE  (familiar object)
                                                      │ no match → ALARM (buzzer)
```

Running vision only after the cheap distance check means the Pi isn't doing
image processing on empty hallways, and the whitelist step means a user's own
cane or bag doesn't set off the alarm.

## Hardware

- Raspberry Pi (with Pi Camera, via `picamera2`)
- HC-SR04 ultrasonic distance sensor
- Passive buzzer
- Momentary push button

### Wiring (BCM pin numbers)

| Component | Pin |
|---|---|
| Push button | GPIO 17 |
| Ultrasonic TRIG | GPIO 23 |
| Ultrasonic ECHO | GPIO 24 |
| Buzzer | GPIO 18 |

## Controls

A single button drives the whole device:

- **Short press** — toggle the system on/off (one buzzer beep confirms).
- **Long press (2 s)** — recalibrate: recapture the empty-floor reference and
  re-register the whitelist object. Useful when the user moves to a new room
  or lighting changes.

On startup the device walks you through calibration: point it at empty floor,
then hold up the object you want treated as safe.

## Running it

On the Raspberry Pi (Raspberry Pi OS Bookworm — `picamera2` and `RPi.GPIO`
ship with the OS image):

```bash
pip install -r requirements.txt   # opencv-python + numpy if not already present
python main.py
```

## How detection works

- **Distance** ([`get_distance`](main.py)) — fires the HC-SR04 five times,
  drops the high/low outliers, and averages the middle three. Every echo wait
  is timeout-guarded so a missed pulse can't hang the loop.
- **Obstacle detection** ([`get_object_mask`](main.py)) — absolute difference
  between the live frame and a stored empty-floor image, thresholded and
  dilated into a mask. The pixel count of that mask decides whether an object
  is really there.
- **Whitelist matching** ([`check_object`](main.py)) — the obstacle region's
  HSV hue/saturation histogram is correlated against a user-registered object.
  A correlation above `WHITELIST_TOLERANCE` (0.7) is treated as a familiar,
  safe object; anything else alarms.

Tunable thresholds (distance, pixel sensitivity, whitelist tolerance) are
constants at the top of [`main.py`](main.py).

## Repo layout

```
main.py          # the production pipeline (run this)
requirements.txt
prototypes/      # earlier detection approaches we tried (MOG2, Canny, floor-diff)
                 # see prototypes/README.md for what each was and why we moved on
```

The [`prototypes/`](prototypes/) folder documents the design iteration — we
went through background subtraction and edge detection before landing on
floor-reference differencing plus histogram whitelisting.

## Authors

<!-- TODO: list the team members -->
