import os
import math
import time
import threading
from collections import deque
import urllib.request

import cv2
import serial
import mediapipe as mp
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.core.base_options import BaseOptions

# ===================== SETTINGS =====================
COM_PORT = 'COM3'          # Change to your Arduino's port
BAUD_RATE = 115200
CAMERA_INDEX = 0

#for prototype we set this to false, true would mean that the camera is already implemented ON the gimbal, false means that it is using the laptop camera still
CAMERA_ON_GIMBAL = False
# --- Camera ---
FRAME_WIDTH, FRAME_HEIGHT = 640, 480
CAMERA_FPS = 60            #run at 60FPS
CAMERA_HFOV_DEG = 60       # change to match camera FOV
MANUAL_EXPOSURE = None

USE_DIRECTSHOW = True

#gimbal mode tuning for when CAMERA_ON_GIMBAL is changed to true
RESPONSE = 0.45            # Fraction of the error corrected each frame (0.2 calm, 0.6 aggressive)
LATENCY = 0.10             # Seconds between a servo command and the camera seeing it move.
                           # Raise if it overshoots/wobbles, lower if it feels sluggish.
MAX_STEP_DEG = 8           # Max degrees per frame (protects gears from whiplash)
DEADZONE_DEG = 1.0         # Ignore errors smaller than this (stops buzzing)

# --- Fixed camera mode tuning (CAMERA_ON_GIMBAL = False) ---
PAN_RANGE = (40, 140)      # Servo angle at the left / right edge of the image
TILT_RANGE = (50, 130)     # Servo angle at the top / bottom edge of the image

# --- Prediction (aim where the hand is GOING, not where it was) ---
LEAD_TIME = 0.06           # Seconds to aim ahead. 0 = off, 0.05-0.12 typical.
                           # Raise if the laser trails fast waves, lower if it overshoots.
MAX_LEAD_FRAC = 0.15       # Cap on how far ahead to aim (fraction of the image width)

# --- Smoothing (One Euro filter) ---
FILTER_MIN_CUTOFF = 1.5    # Lower = smoother when hand is still (more lag)
FILTER_BETA = 0.05         # Higher = less lag when hand moves fast

# --- Lost hand behavior ---
HOLD_TIME = 0.5            # Seconds to hold position after the hand disappears
COAST_TIME = 0.0           # Experimental: if the hand blinks out mid-motion, keep moving the
                           # laser along its last direction for this long (try 0.1). Helps
                           # mid-swing, but overshoots if the hand drops out while reversing.
RETURN_SPEED = 60          # Degrees per second when drifting back to center

ANGLE_MIN, ANGLE_MAX = 15, 165
CENTER = 90
# Hand detection sensitivity. Lower = loses the hand less often during fast motion,
# but may occasionally lock onto things that aren't hands. 0.3-0.6 is reasonable.
DETECTION_CONFIDENCE = 0.4
TRACKING_CONFIDENCE = 0.3
# ====================================================

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")

# Palm center = wrist, index base, middle base, pinky base
PALM_POINTS = [0, 5, 9, 17]

HAND_LINES = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),
]


# ---------------------------------------------------------------
# Camera reader on its own thread: always hands back the NEWEST
# frame, so the tracker never works on stale, buffered images.
# ---------------------------------------------------------------
class CameraStream:
    def __init__(self, index):
        # DirectShow opens faster and allows MJPG / high FPS on Windows
        if os.name == 'nt' and USE_DIRECTSHOW:
            self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(index)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if MANUAL_EXPOSURE is not None:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual on DirectShow
            self.cap.set(cv2.CAP_PROP_EXPOSURE, MANUAL_EXPOSURE)

        # How fast the camera itself is delivering frames
        self.cam_fps = 0.0
        self._last_frame_time = None

        self.frame = None
        self.frame_id = 0
        self.running = self.cap.isOpened()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update, daemon=True)
        if self.running:
            self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.005)
                continue
            now = time.monotonic()
            if self._last_frame_time is not None:
                dt = max(now - self._last_frame_time, 1e-3)
                self.cam_fps = 0.9 * self.cam_fps + 0.1 * (1.0 / dt)
            self._last_frame_time = now
            with self.lock:
                self.frame = frame
                self.frame_id += 1

    def read(self, last_id):
        """Wait for a frame newer than last_id, return (frame, id)."""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with self.lock:
                if self.frame is not None and self.frame_id != last_id:
                    return self.frame.copy(), self.frame_id
            time.sleep(0.001)
        return None, last_id

    def is_opened(self):
        return self.running

    def release(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1)
        self.cap.release()


# ---------------------------------------------------------------
# One Euro filter: heavy smoothing when the hand is still,
# almost no lag when it moves fast. Better than plain averaging.
# ---------------------------------------------------------------
class OneEuroFilter:
    def __init__(self, min_cutoff, beta, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def __call__(self, x, t):
        if self.x_prev is None:
            self.x_prev, self.t_prev = x, t
            return x
        dt = max(t - self.t_prev, 1e-3)
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev, self.dx_prev, self.t_prev = x_hat, dx_hat, t
        return x_hat


def get_model():
    if os.path.exists(MODEL_PATH):
        return MODEL_PATH
    print("Downloading hand model (one time only)...")
    try:
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("Model downloaded.")
        return MODEL_PATH
    except Exception as e:
        print(f"Could not download the model: {e}")
        print(f"Download it manually from:\n  {MODEL_URL}\nand save it as:\n  {MODEL_PATH}")
        exit()


def connect_arduino():
    try:
        ser = serial.Serial(port=COM_PORT, baudrate=BAUD_RATE, timeout=0.1, write_timeout=0)
        time.sleep(2)  # Arduino resets when the port opens
        print("Connected to Arduino successfully!")
        return ser
    except Exception as e:
        print(f"Error connecting to Arduino: {e}")
        exit()


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def step_toward(value, target, max_step):
    diff = target - value
    if abs(diff) <= max_step:
        return target
    return value + math.copysign(max_step, diff)


def draw_hand(frame, landmarks, w, h):
    points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]
    for a, b in HAND_LINES:
        cv2.line(frame, points[a], points[b], (255, 255, 255), 2)
    for p in points:
        cv2.circle(frame, p, 4, (0, 200, 0), -1)


def main():
    options = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=get_model()),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=DETECTION_CONFIDENCE,
        min_hand_presence_confidence=TRACKING_CONFIDENCE,
        min_tracking_confidence=TRACKING_CONFIDENCE,
    )
    detector = vision.HandLandmarker.create_from_options(options)

    arduino = connect_arduino()

    cam = CameraStream(CAMERA_INDEX)
    if not cam.is_opened():
        print("Error: could not open camera.")
        arduino.close()
        detector.close()
        return

    # Angles are floats internally (no rounding drift); rounded only when sent
    pan, tilt = float(CENTER), float(CENTER)
    last_sent = None
    history = deque()  # (time, pan, tilt) of recent commands, oldest first
    filt_x = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA)
    filt_y = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA)
    prev_tx = prev_ty = None
    prev_t = None
    vel_x = vel_y = 0.0
    aim_x = aim_y = None

    start = time.monotonic()
    last_ts = -1
    last_seen = None
    prev_loop = time.monotonic()
    frame_id = 0
    fps = 0.0

    mode = "gimbal" if CAMERA_ON_GIMBAL else "fixed camera"
    print(f"Hand Tracking started ({mode} mode). Press 'q' to quit.")

    try:
        while True:
            frame, frame_id = cam.read(frame_id)
            if frame is None:
                print("Camera frame not received.")
                break

            now = time.monotonic()
            dt = max(now - prev_loop, 1e-3)
            prev_loop = now
            fps = 0.9 * fps + 0.1 * (1.0 / dt)

            frame = cv2.flip(frame, 1)  # Mirror image
            h, w, _ = frame.shape
            cx, cy = w / 2, h / 2
            deg_per_px = CAMERA_HFOV_DEG / w

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            ts = max(int((now - start) * 1000), last_ts + 1)
            last_ts = ts
            result = detector.detect_for_video(mp_image, ts)

            hand_detected = bool(result.hand_landmarks)

            if hand_detected:
                last_seen = now
                lms = result.hand_landmarks[0]
                draw_hand(frame, lms, w, h)

                raw_x = sum(lms[i].x for i in PALM_POINTS) / len(PALM_POINTS) * w
                raw_y = sum(lms[i].y for i in PALM_POINTS) / len(PALM_POINTS) * h
                tx = filt_x(raw_x, now)
                ty = filt_y(raw_y, now)
                cv2.circle(frame, (int(tx), int(ty)), 10, (0, 0, 255), -1)

                # Hand velocity (pixels per second), lightly smoothed.
                # Uses the time since the last real sighting, so a short
                # dropout doesn't create a fake speed spike.
                if prev_tx is not None:
                    span = max(now - prev_t, 1e-3)
                    vel_x = 0.5 * vel_x + 0.5 * (tx - prev_tx) / span
                    vel_y = 0.5 * vel_y + 0.5 * (ty - prev_ty) / span
                prev_tx, prev_ty, prev_t = tx, ty, now

                if CAMERA_ON_GIMBAL:
                    # Error in degrees: how far the servos need to turn
                    err_pan = (tx - cx) * deg_per_px
                    err_tilt = (ty - cy) * deg_per_px

                    # The camera image is a little old: subtract the movement
                    # already commanded that the camera hasn't seen yet.
                    # This is what lets it be fast without overshooting.
                    old_pan, old_tilt = pan, tilt
                    for t_cmd, p_cmd, tl_cmd in history:
                        if now - t_cmd <= LATENCY:
                            old_pan, old_tilt = p_cmd, tl_cmd
                            break
                    err_pan -= pan - old_pan
                    err_tilt -= tilt - old_tilt

                    if abs(err_pan) > DEADZONE_DEG:
                        pan += clamp(err_pan * RESPONSE, -MAX_STEP_DEG, MAX_STEP_DEG)
                    if abs(err_tilt) > DEADZONE_DEG:
                        tilt += clamp(err_tilt * RESPONSE, -MAX_STEP_DEG, MAX_STEP_DEG)
                else:
                    # Aim slightly ahead of the hand to cancel out system lag
                    max_lead = MAX_LEAD_FRAC * w
                    aim_x = tx + clamp(vel_x * LEAD_TIME, -max_lead, max_lead)
                    aim_y = ty + clamp(vel_y * LEAD_TIME, -max_lead, max_lead)
                    aim_x = clamp(aim_x, 0, w)
                    aim_y = clamp(aim_y, 0, h)
                    cv2.circle(frame, (int(aim_x), int(aim_y)), 6, (255, 0, 255), 2)

                    # Map the aim point straight to an angle
                    pan = PAN_RANGE[0] + (aim_x / w) * (PAN_RANGE[1] - PAN_RANGE[0])
                    tilt = TILT_RANGE[0] + (aim_y / h) * (TILT_RANGE[1] - TILT_RANGE[0])

                pan = clamp(pan, ANGLE_MIN, ANGLE_MAX)
                tilt = clamp(tilt, ANGLE_MIN, ANGLE_MAX)
            elif (not CAMERA_ON_GIMBAL and aim_x is not None and last_seen is not None
                  and now - last_seen <= COAST_TIME):
                # Hand blinked out (usually motion blur): keep the laser moving
                # the way the hand was going, slowing down, until it reappears
                fade = (1.0 - (now - last_seen) / COAST_TIME) ** 2
                aim_x = clamp(aim_x + vel_x * fade * dt, 0, w)
                aim_y = clamp(aim_y + vel_y * fade * dt, 0, h)
                cv2.circle(frame, (int(aim_x), int(aim_y)), 6, (0, 165, 255), 2)
                pan = clamp(PAN_RANGE[0] + (aim_x / w) * (PAN_RANGE[1] - PAN_RANGE[0]), ANGLE_MIN, ANGLE_MAX)
                tilt = clamp(TILT_RANGE[0] + (aim_y / h) * (TILT_RANGE[1] - TILT_RANGE[0]), ANGLE_MIN, ANGLE_MAX)
            else:
                filt_x.reset()
                filt_y.reset()
                prev_tx = prev_ty = prev_t = None
                vel_x = vel_y = 0.0
                aim_x = aim_y = None
                # Hold briefly (hand might just have blinked out), then drift home
                if last_seen is None or now - last_seen > HOLD_TIME:
                    pan = step_toward(pan, CENTER, RETURN_SPEED * dt)
                    tilt = step_toward(tilt, CENTER, RETURN_SPEED * dt)

            history.append((now, pan, tilt))
            while history and now - history[0][0] > LATENCY + 0.5:
                history.popleft()

            # Send with one decimal place: the Arduino turns this into
            # microsecond pulses, ~10x finer than whole degrees
            current = (round(pan, 1), round(tilt, 1))
            if current != last_sent:
                arduino.write(f"{current[0]:.1f},{current[1]:.1f}\n".encode())
                last_sent = current

            if hand_detected:
                status = "TRACKING"
            elif aim_x is not None:
                status = "COASTING"
            elif last_seen and now - last_seen <= HOLD_TIME:
                status = "HOLDING"
            else:
                status = "SEARCHING"
            cv2.drawMarker(frame, (int(cx), int(cy)), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            cv2.putText(frame, f"{status}  Pan:{current[0]:.1f}  Tilt:{current[1]:.1f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Camera FPS = how fast the webcam delivers frames
            # Track FPS  = how fast the script processes them
            cv2.putText(frame, f"Camera FPS:{cam.cam_fps:.0f}  Track FPS:{fps:.0f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow('Hand Tracker View', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        try:
            arduino.write(f"{CENTER},{CENTER}\n".encode())
            time.sleep(0.1)
        except Exception:
            pass
        detector.close()
        cam.release()
        cv2.destroyAllWindows()
        arduino.close()
        print("Tracking closed safely.")


if __name__ == "__main__":
    main()
