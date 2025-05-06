"""
AprilTag Recognition System for Robot Navigation
Optimized for Raspberry Pi 4 with web-based visualization
"""

import cv2
import time
import os
import numpy as np
from threading import Thread, Lock, Event
import queue
import serial
import signal
import sys
from flask import Flask, Response, render_template_string

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector

# === Camera Configuration ===
CAMERA_WIDTH = 1280  # Higher resolution for better detection at distance
CAMERA_HEIGHT = 720
FOCAL_LENGTH_PX = 1600  # Adjusted focal length for new resolution
TAG_REAL_SIZE_CM = 15
DISTANCE_THRESHOLD = 45
CENTER_TOLERANCE_RATIO = 0.15

# === Arduino Communication ===
SEND_TO_ARDUINO = False
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# === Performance Settings ===
FRAME_QUEUE_SIZE = 5  # Increased queue size for smoother processing
SKIP_FRAMES = 1  # Process every frame on Pi 4
USE_THREADING = True
SHOW_FEED = True
VERBOSE = True

# === Global Variables ===
frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
frame_lock = Lock()
last_direction = None
global_frame = None
shutdown_event = Event()  # For coordinating clean shutdown

# Initialize Flask app with minimal logging
app = Flask(__name__)
app.logger.setLevel(40)  # Set to ERROR level (40)

# HTML template for the web interface
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>AprilTag Detection</title>
    <style>
        body {
            margin: 0;
            background: #000;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
        }
        img {
            max-width: 100%;
            height: auto;
        }
    </style>
</head>
<body>
    <img src="{{ url_for('video_feed') }}">
</body>
</html>
"""

@app.route('/')
def index():
    """Serve the main page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    """Generate frames for the web stream"""
    while not shutdown_event.is_set():
        with frame_lock:
            if global_frame is not None:
                try:
                    ret, buffer = cv2.imencode('.jpg', global_frame)
                    if ret:
                        frame_data = buffer.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
                except Exception as e:
                    print(f"Error generating frame: {e}")
        time.sleep(0.1)  # Reduce frame rate to avoid overwhelming the Pi

def setup_serial():
    """Initialize serial connection to Arduino"""
    if not SEND_TO_ARDUINO:
        return None
    try:
        arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for Arduino reset
        print("Arduino connected successfully")
        return arduino
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return None

def estimate_tag_size(corners):
    """Calculate tag size based on corner positions"""
    side_lengths = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
    return np.mean(side_lengths)

def get_direction(detection, frame_width, distance_threshold):
    """Determine movement direction based on tag position and distance"""
    center_x = detection.center[0]
    tag_size = estimate_tag_size(detection.corners)
    frame_center = frame_width // 2
    center_tolerance = frame_width * CENTER_TOLERANCE_RATIO

    # Check distance first
    if tag_size >= distance_threshold:
        return 'S', tag_size

    # Check horizontal position
    if center_x < frame_center - center_tolerance:
        return 'L', tag_size
    elif center_x > frame_center + center_tolerance:
        return 'R', tag_size
    return 'F', tag_size

class StreamCapture:
    def __init__(self, picam):
        self.picam = picam
        self.thread = Thread(target=self._capture_frames, daemon=True)
        self.running = False

    def start(self):
        self.running = True
        self.thread.start()

    def _capture_frames(self):
        while self.running and not shutdown_event.is_set():
            if not frame_queue.full():
                try:
                    frame = self.picam.capture_array()
                    # Apply rotation if needed
                    # Uncomment if camera is mounted in a different orientation
                    # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    frame_queue.put(frame)
                except Exception as e:
                    print(f"Camera capture error: {e}")
                    time.sleep(0.1)  # Sleep on error to avoid rapid error loops
            time.sleep(0.001)

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            try:
                self.thread.join(timeout=1.0)  # Wait with timeout
            except Exception:
                pass  # Ignore join errors on shutdown

def cleanup_camera_resources():
    """Attempt to clean up any lingering camera processes"""
    try:
        import subprocess
        # Try to kill any processes that might be using the camera
        subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], stderr=subprocess.DEVNULL)
        time.sleep(1)  # Allow time for processes to terminate
        return True
    except Exception as e:
        print(f"Warning: Failed to clean up camera resources: {e}")
        return False

def setup_camera():
    """Initialize Pi Camera with optimized settings for Pi 4"""
    try:
        picam2 = Picamera2()
        
        # Advanced camera configuration with settings optimized for AprilTag detection
        config = picam2.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "RGB888"},
            controls={
                "FrameDurationLimits": (33333, 33333),  # Force 30fps
                "AwbEnable": True,  # Auto white balance for consistent colors
                "AeEnable": True,   # Auto exposure for adapting to lighting changes
                "AnalogueGain": 1.0, # Base gain level
                "ExposureTime": 16000,  # Faster exposure to reduce motion blur
                "Sharpness": 10.0,      # Increase sharpness for better tag edges
                "Contrast": 1.2,        # Slightly higher contrast for tag detection
                "NoiseReductionMode": 2 # Balanced noise reduction
            },
            buffer_count=4
        )
        
        picam2.configure(config)
        picam2.start()
        
        # Allow camera to settle with optimal parameters
        time.sleep(0.5)
        
        return picam2
    except RuntimeError as e:
        if "Failed to acquire camera: Device or resource busy" in str(e) or "Camera __init__ sequence did not complete" in str(e):
            print("Camera is busy. Attempting to free resources...")
            if cleanup_camera_resources():
                print("Please restart the program now.")
            else:
                print("Failed to clean up resources. Please try rebooting the system.")
        else:
            print(f"Camera setup error: {e}")
        return None
    except Exception as e:
        print(f"Unexpected camera error: {e}")
        return None

def preprocess_image(frame):
    """Optimize image for tag detection"""
    # Convert to grayscale - AprilTags work best with grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    
    # Apply adaptive histogram equalization with optimized parameters
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4,4))
    gray = clahe.apply(gray)
    
    # Sharpen image to enhance edges (critical for tag detection)
    kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    gray = cv2.filter2D(gray, -1, kernel)
    
    # Optional: Apply very mild blur to reduce noise while preserving edges
    # Uncomment if tags have noisy edges in certain lighting conditions
    # gray = cv2.GaussianBlur(gray, (3,3), 0.3)
    
    return gray

def update_frame_display(frame):
    """Update the display frame with thread safety"""
    global global_frame
    with frame_lock:
        global_frame = frame.copy()

def signal_handler(sig, frame):
    """Handle shutdown signals gracefully"""
    print("Shutting down...")
    shutdown_event.set()
    sys.exit(0)

def flask_server():
    """Run Flask server with proper error handling"""
    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except Exception as e:
        print(f"Flask server error: {e}")

def main():
    global last_direction, global_frame
    last_print_time = 0
    PRINT_INTERVAL = 0.5  # Only print debug info every 0.5 seconds
    
    # Setup tag tracking
    last_tag_positions = {}
    tag_tracking_threshold = 50  # pixels

    # Setup signal handler for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not IS_RASPBERRY_PI:
        print("This script requires a Raspberry Pi to run")
        return

    # Initialize hardware
    arduino = setup_serial()
    picam2 = setup_camera()
    
    if not picam2:
        print("Failed to initialize camera. Exiting.")
        return
        
    detector = Detector(
        families="tag36h11",
        nthreads=4,           # Use all cores on Pi 4
        quad_decimate=1.0,    # No decimation for max accuracy with higher resolution
        quad_sigma=0.2,       # Reduced blur for sharper edges
        refine_edges=True,    # Important for accurate corner detection
        decode_sharpening=0.5,# Increased sharpening for better detection
        debug=False
    )

    stream = StreamCapture(picam2)
    stream.start()
    
    # Create a status frame to show at startup
    status_frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
    cv2.putText(status_frame, "Starting camera...", (50, CAMERA_HEIGHT//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    update_frame_display(status_frame)
    
    if SHOW_FEED:
        flask_thread = Thread(target=flask_server, daemon=True)
        flask_thread.start()
    
    frame_count = 0
    last_frame_time = 0
    FRAME_INTERVAL = 1/30  # Target 30fps processing
    last_command_time = 0
    COMMAND_INTERVAL = 0.2  # Minimum time between commands

    try:
        while not shutdown_event.is_set():
            current_time = time.time()
            
            # Enforce frame rate limit to avoid overwhelming the Pi
            if current_time - last_frame_time < FRAME_INTERVAL:
                time.sleep(0.001)  # Small sleep to reduce CPU usage
                continue
                
            last_frame_time = current_time
            
            try:
                frame = frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # Skip frame processing based on SKIP_FRAMES setting
            frame_count += 1
            if frame_count % SKIP_FRAMES != 0:
                if SHOW_FEED:
                    update_frame_display(frame)  # Still show unprocessed frames
                continue

            # Process frame with optimized pipeline
            processed = preprocess_image(frame)
            detections = detector.detect(processed)

            if detections:
                # Track multiple tags
                for detection in detections:
                    tag_id = detection.tag_id
                    center = detection.center
                    
                    # Update tracking dictionary
                    last_tag_positions[tag_id] = {
                        'center': center,
                        'time': current_time,
                        'corners': detection.corners
                    }
                
                # Find the most relevant tag (closest to center, or by preferred ID if needed)
                main_tag = min(detections, key=lambda d: abs(d.center[0] - frame.shape[1]/2))
                tag_size_px = estimate_tag_size(main_tag.corners)
                direction, _ = get_direction(main_tag, frame.shape[1], DISTANCE_THRESHOLD)
                distance_cm = (FOCAL_LENGTH_PX * TAG_REAL_SIZE_CM) / tag_size_px

                # Only send commands at intervals and when direction changes
                if current_time - last_command_time >= COMMAND_INTERVAL:
                    if arduino:
                        tag_data = f"TAG:{main_tag.tag_id},{distance_cm:.1f},{direction}\n"
                        arduino.write(tag_data.encode())
                        arduino.flush()
                        last_direction = direction
                    last_command_time = current_time

                # Print debug info at intervals
                if VERBOSE and current_time - last_print_time >= PRINT_INTERVAL:
                    print(f"Tag {main_tag.tag_id} | Dist: {distance_cm:.1f}cm | Dir: {direction}")
                    last_print_time = current_time

                if SHOW_FEED:
                    # Draw all detected tags with different colors based on tracking history
                    for detection in detections:
                        pts = detection.corners.astype(int)
                        cv2.polylines(frame, [pts.reshape((-1,1,2))], True, (0,255,0), 2)
                        cv2.putText(frame, f"ID:{detection.tag_id}", 
                                  tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
                    
                    # Highlight the main tag
                    pts = main_tag.corners.astype(int)
                    cv2.polylines(frame, [pts.reshape((-1,1,2))], True, (0,255,255), 3)
                    cv2.putText(frame, f"MAIN ID:{main_tag.tag_id} D:{int(distance_cm)}cm",
                              (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            else:
                # Clean up old tag positions
                current_tags = list(last_tag_positions.keys())
                for tag_id in current_tags:
                    if current_time - last_tag_positions[tag_id]['time'] > 2.0:  # Remove after 2 seconds
                        del last_tag_positions[tag_id]
                
                if current_time - last_print_time >= PRINT_INTERVAL:
                    if VERBOSE:
                        print("No tags detected")
                    last_print_time = current_time
                
                if current_time - last_command_time >= COMMAND_INTERVAL:
                    if arduino:
                        arduino.write(b'S')  # Stop command when no tags detected
                        arduino.flush()
                    last_command_time = current_time

            if SHOW_FEED:
                # Add overlay with detection stats
                cv2.putText(frame, f"Tags: {len(detections)}", 
                          (frame.shape[1]-150, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                update_frame_display(frame)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        shutdown_event.set()  # Signal all threads to stop
        stream.stop()
        if picam2:
            try:
                picam2.stop()
            except Exception:
                pass
        if arduino:
            arduino.close()
        print("Shutdown complete")

if __name__ == '__main__':
    main()