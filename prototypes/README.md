# Prototypes

The detection approaches we tried before settling on the floor-reference +
whitelist-histogram pipeline in [`../main.py`](../main.py). Kept for reference —
each is a self-contained script that ran on the device.

| File | Detection approach | Why we moved on |
|---|---|---|
| `01_mog2_background_subtraction.py` | OpenCV `MOG2` background subtractor + contour area | Adaptive background drifted as the wearer moved; static obstacles faded into the model and stopped triggering. |
| `02_canny_edge_area.py` | Canny edges + largest-contour area, with size gating (large = furniture = safe, medium = obstacle) | Size alone was a poor proxy for danger; lighting changed edge density too much to tune a stable threshold. |
| `03_floor_reference_diff.py` | Absolute difference vs. a captured empty-floor reference + changed-pixel count | Reliable obstacle *detection*, but no way to tell a known-safe object from a hazard — every change alarmed. |

The final pipeline keeps the floor-reference differencing from (3) and adds an
HSV colour-histogram **whitelist** so familiar objects don't false-alarm.
