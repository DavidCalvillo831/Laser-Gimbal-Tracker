# Laptop Webcam Tracker (Prototype)

First working version of the tracker. The laptop webcam finds your hand, and an Arduino points the pan-tilt gimbal at it.

<!-- ![demo](../../docs/images/demo.gif) -->

## Files

| File | Runs on | What it does |
|---|---|---|
| `Hand_Tracker.py` | Laptop | Detects the hand with MediaPipe, smooths and predicts its motion, sends `pan,tilt` angles over serial |
| `Pan_Tilt_Tracker/Pan_Tilt_Tracker.ino` | Arduino | Receives angles, converts them to microsecond pulses, and glides the servos smoothly |
| `requirements.txt` | Laptop | Python packages |

## Wiring

- Servo signal wires → pan on pin 10, tilt on pin 9 (Arduino Uno)
- Servo power → LM2596 buck output set to 5.0–5.5V (fed from a 12V adapter)
- Buck ground, servo grounds, and Arduino GND all tied together (required)
- 470–1000 µF capacitor across the servo 5V and ground, near the servos
- Arduino powered over USB from the laptop

<!-- Add a wiring photo: ![wiring](../../docs/images/wiring.jpg) -->

## Running it

1. Upload `Pan_Tilt_Tracker.ino` to the Arduino.
2. `pip install -r requirements.txt`
3. Set `COM_PORT` at the top of `Hand_Tracker.py` to your Arduino's port.
4. `python Hand_Tracker.py`. The hand model downloads on the first run.

## Settings worth knowing (top of `Hand_Tracker.py`)

| Setting | What it does |
|---|---|
| `CAMERA_ON_GIMBAL` | `False` for a fixed camera like a laptop webcam |
| `RESPONSE`, `LATENCY` | How aggressive the tracking is and how far ahead it predicts |
| `DETECTION_CONFIDENCE` | Lower keeps the hand during fast motion, higher avoids false detections |
| `MANUAL_EXPOSURE` | Forces a short exposure so the webcam doesn't slow down in dim light |
| `USE_DIRECTSHOW` | Switches the Windows camera driver; some webcams are faster on one |

## What I learned

- **Servo jitter** came from whole-degree steps and powering servos from the Arduino's 5V pin. Fixed with microsecond pulses and a separate buck supply.
- **Erratic servo motion** was a missing shared ground between the buck and the Arduino.
- **Frame rate** is capped at about 20 FPS by the laptop webcam, not the code. Prediction covers some of that lag; the dedicated XIAO camera is the next step.

## Next

Replace the laptop webcam with the XIAO ESP32-S3 camera stream, then move to the onboard build.
