"""
Optimized AprilTag Recognition System for Robot Navigation
Focused on accurate tag detection without video streaming features
"""

import cv2
import time
import os
import numpy as np
import serial
import signal
import sys
from threading import Event

# Check if running on Raspberry Pi
IS_RASPBERRY_PI = os.path.exists('/sys/firmware/devicetree/base/model')

if IS_RASPBERRY_PI:
    from picamera2 import Picamera2
    from pupil_apriltags import Detector
    from apriltag_communication import ArduinoCommunicator, TagData

# === Configuration ===
# Camera settings
CAMERA_WIDTH = 1640       # Higher resolution for better long-distance detection
CAMERA_HEIGHT = 1232      # Native 4:3 resolution for v2 camera
FOCAL_LENGTH_PX = 1000    # Camera focal length in pixels
TAG_SIZE_MM = 150         # Physical tag size in mm
TAG_SIZE_CM = TAG_SIZE_MM / 10  # Convert to cm for existing code compatibility

# Camera intrinsic parameters (for improved distance calculation)
# These parameters should be calibrated for your specific camera
CAM_FX = 1000.0  # Focal length in x direction (pixels)
CAM_FY = 1000.0  # Focal length in y direction (pixels)
CAM_CX = CAMERA_WIDTH / 2  # Principal point x-coordinate
CAM_CY = CAMERA_HEIGHT / 2  # Principal point y-coordinate

# Arduino communication
ARDUINO_ENABLED = False    # Enable Arduino communication
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Detection settings
VERBOSE = True            # Set to False to reduce console output
DETECTION_INTERVAL = 0.1  # 10 FPS for more reliable processing

# Navigation parameters
CENTER_TOLERANCE = 0.15   # Percentage of frame width for center tolerance
DISTANCE_THRESHOLD = 30   # Distance (cm) threshold for movement decisions

# === Global state ===
shutdown_event = Event()

def setup_camera():
    """Initialize the Raspberry Pi camera with optimal settings for AprilTag detection"""
    if not IS_RASPBERRY_PI:
        return None
    
    try:
        # Initialize camera
        picam = Picamera2()
        
        # Configure camera with settings optimized for tag detection
        config = picam.create_preview_configuration(
            main={"size": (CAMERA_WIDTH, CAMERA_HEIGHT), "format": "BGR888"},
            controls={
                "FrameRate": 15,          # Lower framerate for more stable images
                "AwbEnable": True,        # Auto white balance
                "ExposureTime": 8000,     # 8ms exposure for clearer edges
                "AnalogueGain": 4.0,      # Increased gain for better visibility at distance
                "Sharpness": 15.0,        # Maximum sharpness for clearest tag edges
                "Contrast": 2.0,          # Higher contrast for better black/white distinction
                "Brightness": 0.0,        # Neutral brightness to prevent washout
                "NoiseReductionMode": 0,  # Disable noise reduction to preserve edges
                "AwbMode": 1              # Auto white balance mode (1 = normal)
                # Removed unsupported parameters
            }
        )
        
        # Use IPA (Image Processing Algorithms) file for advanced tuning
        # This loads the IMX219.json tuning parameters automatically
        picam.set_controls({"NoiseReductionMode": 0})  # Explicitly disable noise reduction for tag edges
        
        picam.configure(config)
        picam.start()
        
        # Allow the camera to adjust to the scene
        time.sleep(0.5)  # Short delay for camera initialization
        
        # Fine-tune the exposure parameters - removed unsupported parameters
        picam.set_controls({
            "FrameDurationLimits": (33333, 66666)  # Set min/max frame duration based on imx219.json
        })
        
        return picam
        
    except Exception as e:
        print(f"Camera setup failed: {e}")
        # Try to clean up any camera resources
        try:
            import subprocess
            subprocess.run(['sudo', 'pkill', '-f', 'libcamera'], stderr=subprocess.DEVNULL)
        except:
            pass
        return None

def setup_serial():
    """Initialize serial connection to Arduino"""
    if not ARDUINO_ENABLED:
        return None
    
    try:
        communicator = ArduinoCommunicator(SERIAL_PORT, BAUD_RATE)
        if communicator.connect():
            print("Arduino connected successfully")
            return communicator
        else:
            print("Failed to connect to Arduino")
            return None
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        return None

def estimate_tag_size(corners):
    """Calculate tag size in pixels based on corner positions with robust metrics.
    
    Args:
        corners: List of four corner points of the AprilTag
        
    Returns:
        Average side length of the tag in pixels
    """
    # Calculate all side lengths
    side_lengths = []
    for i in range(4):
        # Calculate Euclidean distance between consecutive corners
        side = np.linalg.norm(corners[i] - corners[(i+1) % 4])
        side_lengths.append(side)
    
    # Calculate diagonal lengths for additional robustness
    diagonal1 = np.linalg.norm(corners[0] - corners[2])
    diagonal2 = np.linalg.norm(corners[1] - corners[3])
    
    # Verify if the shape is roughly square (for quality control)
    sides_std = np.std(side_lengths)
    sides_mean = np.mean(side_lengths)
    diag_ratio = max(diagonal1, diagonal2) / min(diagonal1, diagonal2)
    
    # If the tag shape is highly distorted, apply corrections
    if sides_std / sides_mean > 0.2 or diag_ratio > 1.3:
        # For distorted tags, prefer the median of sides and use diagonals for verification
        # Sqrt of 2 is diagonal to side ratio in a perfect square
        estimated_side = np.median(side_lengths)
        estimated_side_from_diag = (diagonal1 + diagonal2) / (2 * np.sqrt(2))
        
        # Weighted average giving more importance to sides for slightly distorted tags
        return 0.7 * estimated_side + 0.3 * estimated_side_from_diag
    else:
        # For clean detections, just use the average side length
        return sides_mean

def calculate_distance(tag_size_px):
    """Calculate distance to AprilTag using a more accurate camera model.
    
    This function uses the known physical size of the tag (150mm) and the
    detected size in pixels to calculate distance using the pinhole camera model.
    
    Args:
        tag_size_px: Estimated tag size in pixels
        
    Returns:
        Distance to tag in centimeters
    """
    # Distance calculation using the pinhole camera model:
    # distance = (focal_length * real_size) / pixel_size
    distance_mm = (CAM_FX * TAG_SIZE_MM) / tag_size_px
    distance_cm = distance_mm / 10.0  # Convert mm to cm
    
    # Apply a small correction for lens distortion effects
    # These values should be calibrated based on actual measurements
    if distance_cm < 50:
        return distance_cm * 1.05  # Closer objects tend to appear slightly further than they are
    elif distance_cm < 150:
        return distance_cm * 1.02  # Mid-range slight correction
    else:
        return distance_cm * 0.98  # Far objects tend to appear slightly closer than they are

def get_direction(detection, frame_width):
    """Determine movement direction based on tag position and size"""
    center_x = detection.center[0]
    tag_size_px = estimate_tag_size(detection.corners)
    distance_cm = calculate_distance(tag_size_px)
    
    # Calculate frame center and tolerance zone
    frame_center = frame_width / 2
    tolerance = frame_width * CENTER_TOLERANCE
    
    # Determine direction based on tag position and distance
    if distance_cm < DISTANCE_THRESHOLD:
        return 'S', distance_cm  # Stop when close enough
    
    if center_x < frame_center - tolerance:
        return 'L', distance_cm  # Turn left
    elif center_x > frame_center + tolerance:
        return 'R', distance_cm  # Turn right
    else:
        return 'F', distance_cm  # Move forward

def preprocess_image(frame):
    """Advanced image preprocessing for better tag detection"""
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply advanced adaptive histogram equalization with higher clip limit
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6,6))
    gray = clahe.apply(gray)
    
    # Apply bilateral filter to preserve edges while reducing noise
    gray = cv2.bilateralFilter(gray, 5, 75, 75)
    
    # Enhance contrast with wider range
    gray = cv2.convertScaleAbs(gray, alpha=1.5, beta=15)
    
    # Apply sharper edge enhancement
    kernel = np.array([[-1,-1,-1], [-1,9.5,-1], [-1,-1,-1]])
    gray = cv2.filter2D(gray, -1, kernel)
    
    return gray

def signal_handler(sig, frame):
    """Handle shutdown signals"""
    print("Shutting down...")
    shutdown_event.set()
    sys.exit(0)

def main():
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if not IS_RASPBERRY_PI:
        print("This script requires a Raspberry Pi to run")
        return

    # Initialize hardware
    camera = setup_camera()
    arduino = setup_serial()
    
    if arduino is None and ARDUINO_ENABLED:
        print("Arduino connection failed, continuing in command display mode")
    
    if not camera:
        print("Failed to initialize camera. Exiting.")
        return
    
    # Initialize AprilTag detector with optimal settings for maximum range
    # Using only parameters supported by the installed version of pupil_apriltags
    detector = Detector(
        families="tag36h11",
        nthreads=4,           # Use all cores on Pi 4
        quad_decimate=1.0,    # No decimation for maximum range
        quad_sigma=0.6,       # Slightly higher blur for better detection at distance
        refine_edges=True,
        decode_sharpening=0.7,# Increased sharpening for cleaner tag reading
        debug=False
    )
    
    # Command smoothing variables
    last_commands = []
    MAX_HISTORY = 3
    
    # Detection enhancement variables
    detection_confidence_threshold = 0.5  # Min confidence to accept a detection
    tag_history = {}  # Track recent tag positions for prediction
    
    # Timing variables
    last_command_time = 0
    command_interval = 0.1  # 100ms between commands
    last_frame_time = 0
    
    print("AprilTag detection system running...")
    
    try:
        while not shutdown_event.is_set():
            current_time = time.time()
            
            # Limit frame rate to avoid overwhelming the CPU
            if current_time - last_frame_time < DETECTION_INTERVAL:
                time.sleep(0.001)  # Short sleep to prevent CPU hogging
                continue
                
            last_frame_time = current_time
            
            # Capture frame
            try:
                frame = camera.capture_array("main")
                if frame is None:
                    print("Error: Empty frame captured")
                    time.sleep(0.1)
                    continue
            except Exception as e:
                print(f"Camera capture error: {e}")
                time.sleep(0.1)
                continue
            
            # Try multiple preprocessing techniques for challenging conditions
            processed = preprocess_image(frame)
            
            # Detect AprilTags with the primary processing
            detections = detector.detect(processed)
            
            # If no detections with primary method, try alternative processing
            if not detections:
                # Alternative processing for different lighting conditions
                # 1. Higher contrast for low light
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                alt_processed = cv2.convertScaleAbs(gray, alpha=2.0, beta=30)
                detections = detector.detect(alt_processed)
                
                # 2. If still no detections, try adaptive thresholding
                if not detections:
                    adaptive_thresh = cv2.adaptiveThreshold(
                        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                        cv2.THRESH_BINARY, 11, 2
                    )
                    detections = detector.detect(adaptive_thresh)
            
            # Filter by decision margin (detection confidence)
            if detections:
                detections = [d for d in detections if d.decision_margin > detection_confidence_threshold]
                
            # Process each detected tag
            if detections:
                # Find the tag closest to the center of the frame (most likely the target)
                main_tag = min(detections, key=lambda d: abs(d.center[0] - frame.shape[1]/2))
                
                # Update tag history for this tag ID
                if main_tag.tag_id not in tag_history:
                    tag_history[main_tag.tag_id] = []
                
                # Store detection with timestamp
                tag_history[main_tag.tag_id].append({
                    'time': current_time,
                    'center': main_tag.center,
                    'corners': main_tag.corners
                })
                
                # Keep only recent history (last 1 second)
                tag_history[main_tag.tag_id] = [
                    d for d in tag_history[main_tag.tag_id] 
                    if current_time - d['time'] < 1.0
                ]
                
                # Calculate direction and distance
                direction, distance_cm = get_direction(main_tag, frame.shape[1])
                
                # Apply command smoothing
                last_commands.append(direction)
                if len(last_commands) > MAX_HISTORY:
                    last_commands.pop(0)
                
                # Use majority vote for stable commands
                if len(last_commands) == MAX_HISTORY:
                    from collections import Counter
                    direction = Counter(last_commands).most_common(1)[0][0]
                
                # Send command to Arduino at specified intervals
                current_time = time.time()
                if arduino and current_time - last_command_time >= command_interval:
                    tag_data = TagData(
                        tag_id=main_tag.tag_id, 
                        distance=distance_cm, 
                        direction=direction
                    )
                    arduino.send_tag_data(tag_data)
                    last_command_time = current_time
                elif arduino is None and current_time - last_command_time >= command_interval:
                    # Display command indications when Arduino is not connected
                    command_text = {
                        'F': "FORWARD",
                        'B': "BACKWARD",
                        'L': "LEFT",
                        'R': "RIGHT",
                        'S': "STOP"
                    }.get(direction, "UNKNOWN")
                    
                    print(f"COMMAND: {command_text} | Tag {main_tag.tag_id} | Distance: {distance_cm:.1f}cm")
                    last_command_time = current_time
                
                # Print detection information with enhanced details
                if VERBOSE:
                    print(f"Tag {main_tag.tag_id} | Dist: {distance_cm:.1f}cm | Dir: {direction} | " 
                          f"Conf: {main_tag.decision_margin:.3f} | "
                          f"Size: {estimate_tag_size(main_tag.corners):.1f}px")
                    
            else:
                # No tags detected
                last_commands = []  # Clear command history
                
                # Send stop command when no tags are visible
                current_time = time.time()
                if arduino and current_time - last_command_time >= command_interval:
                    arduino.send_stop()
                    last_command_time = current_time
                elif arduino is None and current_time - last_command_time >= command_interval:
                    # Display stop command when no tags are detected and Arduino is not connected
                    print("COMMAND: STOP | No tags detected")
                    last_command_time = current_time
            
            # Brief pause to reduce CPU load
            time.sleep(0.001)
            
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Clean shutdown
        shutdown_event.set()
        
        if camera:
            try:
                camera.stop()
                print("Camera stopped")
            except:
                pass
                
        if arduino:
            arduino.disconnect()
            print("Serial connection closed")
        
        print("System shutdown complete")

if __name__ == "__main__":
    main()