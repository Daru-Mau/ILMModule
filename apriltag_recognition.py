import cv2
import time
from picamera2 import Picamera2
from pupil_apriltags import Detector
import serial
import numpy as np


FOCAL_LENGTH_PX = 1575  # Approx. camera focal length in pixels

# Calibration of the camera:
CALIBRATION_MODE = False
TAG_REAL_SIZE_CM = 15  # 15cm side for distance estimation
CALIBRATION_DISTANCE_CM = 100  # real distance of the tag from the camera

# Elegoo connection active:
SEND_TO_ARDUINO = True
DISTANCE_THRESHOLD = 100
CENTER_TOLERANCE_RATIO = 0.1
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
VERBOSE = True

SHOW_FEED = True  # Toggle this to False to disable window display

TAG_PHYSICAL_SIZE = 0.15  # meters
last_direction = None


def estimate_tag_size(corners):
    return ((corners[2][0] - corners[0][0])**2 +
            (corners[2][1] - corners[0][1])**2)**0.5


def get_direction(detection, frame_width, distance_threshold):
    center_x = detection.center[0]
    tag_size = estimate_tag_size(detection.corners)
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    if tag_size >= distance_threshold:
        return 'S', tag_size
    if center_x < frame_center - center_tolerance:
        return 'L', tag_size
    elif center_x > frame_center + center_tolerance:
        return 'R', tag_size
    return 'F', tag_size


def setup_serial():
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Arduino connected")
        return arduino
    except Exception as e:
        print(f"Serial error: {e}")
        return None


def setup_camera():
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (224, 224), "format": "RGB888"},
        controls={
            "FrameDurationLimits": (100000, 100000),
            "AnalogueGain": 4.0,
            "ExposureTime": 40000,
            "AwbEnable": 0,
            "Contrast": 1.2
        }
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)
    return picam2


def preprocess_image(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)


arduino = setup_serial() if SEND_TO_ARDUINO else None
picam2 = setup_camera()
detector = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.5,
    quad_sigma=0.8,
    refine_edges=0,
    decode_sharpening=0.7,
    debug=False
)

print("AprilTag tracking started")

try:
    detection_interval = 0.1

    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        processed = preprocess_image(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        detections = detector.detect(processed)

        if detections:
            main_tag = min(detections, key=lambda d: abs(
                d.center[0] - frame.shape[1]/2))
            tag_size_px = estimate_tag_size(main_tag.corners)
            direction, _ = get_direction(
                main_tag, frame.shape[1], DISTANCE_THRESHOLD)

            if CALIBRATION_MODE:
                focal_length = (
                    tag_size_px * CALIBRATION_DISTANCE_CM) / TAG_REAL_SIZE_CM
                print(
                    f"[CALIBRATION] Estimated focal length: {focal_length:.1f} px")
                distance_cm = CALIBRATION_DISTANCE_CM
            else:
                distance_cm = (FOCAL_LENGTH_PX *
                               TAG_REAL_SIZE_CM) / tag_size_px

            detection_interval = max(0.05, 0.2 - tag_size_px / 1000)

            if direction != last_direction:
                if SEND_TO_ARDUINO and arduino:
                    arduino.write(direction.encode())
                    arduino.flush()
                last_direction = direction

            if VERBOSE:
                print(
                    f"Tag {main_tag.tag_id}: {direction} | Size: {tag_size_px:.1f}px | Est. Distance: {distance_cm:.1f} cm")

            # === Draw tag outline and ID ===
            pts = main_tag.corners.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(pts[i]), tuple(
                    pts[(i+1) % 4]), (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{main_tag.tag_id} D:{int(distance_cm)}cm", tuple(pts[0]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        else:
            detection_interval = min(0.5, detection_interval + 0.05)
            if SEND_TO_ARDUINO and arduino:
                arduino.write(b'S')
            if VERBOSE:
                print("Searching...")

        # === Show the camera view ===
        cv2.imshow("AprilTag View", frame)
        if cv2.waitKey(1) == 27:  # ESC key
            break

        time.sleep(detection_interval)
except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    picam2.stop()
    if arduino:
        arduino.close()
    if 'detector' in locals():
        del detector
    if SHOW_FEED:
        cv2.destroyAllWindows()
