import cv2
import serial
import time
import numpy as np

# ===================== SETTINGS =====================
COM_PORT = 'COM3'        # Change to your Arduino's port
BAUD_RATE = 115200
CAMERA_INDEX = 0

#tracks green
LOWER_COLOR = np.array([35, 100, 100])
UPPER_COLOR = np.array([85, 255, 255])

MIN_RADIUS = 15          # Ignore blobs smaller than this (pixels)
DEADZONE = 15           
GAIN = 0.04              # agressiveness
ANGLE_MIN, ANGLE_MAX = 15, 165
CENTER = 90
RETURN_STEP = 1          
# ====================================================


def connect_arduino():
    try:
        ser = serial.Serial(port=COM_PORT, baudrate=BAUD_RATE, timeout=0.1)
        time.sleep(2)  # Arduino resets when the port opens
        print("Connected to Arduino successfully!")
        return ser
    except Exception as e:
        print(f"Error connecting to Arduino: {e}")
        exit()


def step_toward(value, target, step):
    if value < target:
        return min(value + step, target)
    if value > target:
        return max(value - step, target)
    return value


def main():
    arduino = connect_arduino()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Error: could not open camera.")
        arduino.close()
        return

    pan_angle = CENTER
    tilt_angle = CENTER
    last_sent = None

    print("Color Tracking started! Hold a bright green object in front of the camera. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Camera frame not received.")
                break

            frame = cv2.flip(frame, 1)  # Mirror image
            h, w, _ = frame.shape
            center_x, center_y = w // 2, h // 2

            # Blur, convert to HSV, isolate the target color
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color_detected = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest)

                if radius > MIN_RADIUS:
                    color_detected = True
                    tx, ty = int(x), int(y)

                    cv2.circle(frame, (tx, ty), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, (tx, ty), 5, (0, 0, 255), -1)

                    error_x = tx - center_x
                    error_y = ty - center_y

                    if abs(error_x) > DEADZONE:
                        pan_angle += int(error_x * GAIN)
                    if abs(error_y) > DEADZONE:
                        tilt_angle += int(error_y * GAIN)

                    pan_angle = max(ANGLE_MIN, min(ANGLE_MAX, pan_angle))
                    tilt_angle = max(ANGLE_MIN, min(ANGLE_MAX, tilt_angle))

            # Target lost: drift smoothly back to center
            if not color_detected:
                pan_angle = step_toward(pan_angle, CENTER, RETURN_STEP)
                tilt_angle = step_toward(tilt_angle, CENTER, RETURN_STEP)

            # Only send when the angles change (keeps serial traffic light)
            current = (pan_angle, tilt_angle)
            if current != last_sent:
                arduino.write(f"{pan_angle},{tilt_angle}\n".encode())
                last_sent = current

            # On-screen info
            status = "TRACKING" if color_detected else "SEARCHING"
            cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
            cv2.putText(frame, f"{status}  Pan:{pan_angle}  Tilt:{tilt_angle}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow('Color Tracker View', frame)
            cv2.imshow('What the Camera Sees (Target Color is White)', mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        try:
            arduino.write(f"{CENTER},{CENTER}\n".encode())
            time.sleep(0.1)
        except Exception:
            pass
        cap.release()
        cv2.destroyAllWindows()
        arduino.close()
        print("Tracking closed safely.")


if __name__ == "__main__":
    main()
