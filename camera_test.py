from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import time
import threading
import logging
import numpy as np
import io
import os
import signal
import argparse  # Added for command line arguments

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Parse command line arguments
parser = argparse.ArgumentParser(description='Raspberry Pi Camera Test with dual mode support')
parser.add_argument('--direct', action='store_true', help='Run in direct display mode (like libcamera-hello)')
parser.add_argument('--timeout', type=int, default=0, help='Timeout in seconds for direct mode (0 = run until interrupted)')
args = parser.parse_args()

# Create Flask app if in web mode
app = Flask(__name__) if not args.direct else None
frame_lock = threading.Lock()
latest_frame = None
camera_thread = None
running = True

class CameraStream:
    def __init__(self):
        self.picam2 = None
        self.initialized = False
        self.frame_count = 0
    
    def initialize(self):
        try:
            # Close any existing camera first
            self.close()
            
            # Create a new camera instance
            self.picam2 = Picamera2()
            
            # Use the simplest configuration with lower resolution
            video_config = self.picam2.create_still_configuration(
                main={"size": (640, 480)}
            )
            
            self.picam2.configure(video_config)
            self.picam2.start()
            
            # Allow warm-up time
            time.sleep(1)
            
            # Test capture to validate camera is working
            test_frame = self.picam2.capture_array()
            if test_frame is None or len(test_frame.shape) < 2:
                raise Exception("Invalid frame captured during initialization")
                
            self.initialized = True
            logger.info("Camera initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize camera: {str(e)}")
            self.initialized = False
            return False
    
    def get_frame(self):
        if not self.initialized or self.picam2 is None:
            return None
            
        try:
            # Capture a new frame
            frame = self.picam2.capture_array()
            self.frame_count += 1
            
            # Simple validation check
            if frame is None or len(frame.shape) < 2:
                logger.warning("Received invalid frame")
                return None
                
            return frame
        except Exception as e:
            logger.error(f"Error capturing frame: {str(e)}")
            return None
    
    def close(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
                self.picam2 = None
            except Exception as e:
                logger.error(f"Error closing camera: {str(e)}")
            finally:
                self.initialized = False

# Create a camera object
camera = CameraStream()

def generate_frames():
    global latest_frame, running
    
    error_count = 0
    last_successful_capture = time.time()
    
    while running:
        try:
            # Capture a new frame
            frame = camera.get_frame()
            
            if frame is not None:
                # Reset error count on successful capture
                error_count = 0
                last_successful_capture = time.time()
                
                # Encode frame to JPEG
                success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not success:
                    continue
                
                # Update the latest frame
                with frame_lock:
                    latest_frame = buffer.tobytes()
            else:
                error_count += 1
                
                # If we haven't captured a frame successfully for 5 seconds, try to reinitialize
                if time.time() - last_successful_capture > 5:
                    logger.warning("No frames for 5 seconds, attempting camera reinitialization")
                    camera.initialize()
                    last_successful_capture = time.time()  # Reset timer
            
            # Frame rate limiting
            time.sleep(0.1)  # Target around 10 FPS
        
        except Exception as e:
            logger.error(f"Error in frame generation loop: {str(e)}")
            error_count += 1
            time.sleep(0.5)  # Longer delay on error
            
            # If too many consecutive errors, try to reinitialize
            if error_count > 10:
                logger.warning("Too many consecutive errors, reinitializing camera")
                camera.initialize()
                error_count = 0

def start_camera_thread():
    global camera_thread
    if camera_thread is None or not camera_thread.is_alive():
        camera_thread = threading.Thread(target=generate_frames, daemon=True)
        camera_thread.start()

# Signal handler for graceful shutdown
def signal_handler(sig, frame):
    global running
    running = False
    logger.info("Shutdown signal received, stopping camera")
    if camera:
        camera.close()
    # Wait a moment for threads to clean up
    time.sleep(1)
    os._exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Initialize the camera
if not camera.initialize():
    logger.critical("Failed to initialize camera. Check connections and permissions.")
else:
    # If in direct mode, handle display directly with OpenCV
    if args.direct:
        try:
            logger.info("Starting direct display mode (like libcamera-hello)")
            
            # Create a window for display
            window_name = "Camera Preview"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.setWindowTitle(window_name, "Raspberry Pi Camera Preview")
            
            # If timeout is specified and not zero, set a timer to stop
            if args.timeout > 0:
                logger.info(f"Preview will run for {args.timeout} seconds")
                stop_time = time.time() + args.timeout
                
                # Run until timeout or user interruption
                try:
                    while time.time() < stop_time and running:
                        # Capture frame
                        frame = camera.get_frame()
                        
                        if frame is not None:
                            # Display the frame
                            cv2.imshow(window_name, frame)
                            
                            # Break loop if 'q' is pressed
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                        
                        # Brief sleep to control refresh rate
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    logger.info("Direct preview interrupted by user")
            else:
                # Run indefinitely until interrupted
                logger.info("Preview running indefinitely, press 'q' to stop")
                try:
                    while running:
                        # Capture frame
                        frame = camera.get_frame()
                        
                        if frame is not None:
                            # Display the frame
                            cv2.imshow(window_name, frame)
                            
                            # Break loop if 'q' is pressed
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                        
                        # Brief sleep to control refresh rate
                        time.sleep(0.05)
                except KeyboardInterrupt:
                    logger.info("Direct preview interrupted by user")
            
            # Clean up
            cv2.destroyAllWindows()
            camera.close()
            logger.info("Direct preview stopped")
            
        except Exception as e:
            logger.error(f"Error in direct preview mode: {str(e)}")
            camera.close()
        
        # Exit the script when direct mode is done
        logger.info("Direct mode completed, exiting")
        os._exit(0)
    else:
        # Start background thread for web mode
        start_camera_thread()

# Only define Flask routes if not in direct mode
if not args.direct:
    # Simple HTML template with just the video feed
    HTML_TEMPLATE = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Pi Camera Feed</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 20px; text-align: center; }
            h1 { margin-bottom: 20px; }
            .video-container { margin: 0 auto; }
            #status { margin-top: 20px; color: gray; }
        </style>
        <script>
            // Simple status monitoring
            window.onload = function() {
                setInterval(function() {
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('status').innerText = 
                                'Camera status: ' + (data.active ? 'Active' : 'Inactive') + 
                                ' | Frames: ' + data.frames;
                        });
                }, 5000);
            };
        </script>
    </head>
    <body>
        <h1>Raspberry Pi Camera Feed</h1>
        <div class="video-container">
            <img src="{{ url_for('video_feed') }}" alt="Camera Feed">
        </div>
        <div id="status">Camera initializing...</div>
    </body>
    </html>
    """

    @app.route('/')
    def index():
        return render_template_string(HTML_TEMPLATE)

    @app.route('/status')
    def status():
        is_active = camera.initialized and camera_thread and camera_thread.is_alive()
        return {"active": is_active, "frames": camera.frame_count}

    @app.route('/video_feed')
    def video_feed():
        def stream():
            blank_sent = False
            
            while True:
                # Create a fallback frame when needed
                current_frame = None
                with frame_lock:
                    if latest_frame:
                        current_frame = latest_frame
                
                if current_frame:
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + current_frame + b'\r\n')
                    blank_sent = False
                elif not blank_sent:
                    # Send blank frame with error message
                    blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(blank_frame, "No camera signal", (180, 240), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    _, buffer = cv2.imencode('.jpg', blank_frame)
                    yield (b'--frame\r\n'
                          b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    blank_sent = True
                
                # Wait before next frame
                time.sleep(0.1)
        
        return Response(stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Skip Flask server if in direct mode
    if not args.direct:
        try:
            logger.info("Starting camera test server on http://0.0.0.0:5000")
            # Make Flask less verbose with logging
            import logging as flask_logging
            flask_logging.getLogger('werkzeug').setLevel(flask_logging.WARNING)
            
            # Run Flask in threaded mode
            app.run(host='0.0.0.0', port=5000, threaded=True)
        except KeyboardInterrupt:
            logger.info("Server shutdown requested")
        except Exception as e:
            logger.error(f"Error starting server: {str(e)}")
        finally:
            running = False
            if camera:
                camera.close()
            logger.info("Camera test server shutdown complete")
