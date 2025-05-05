from flask import Flask, Response, render_template_string
from picamera2 import Picamera2, Preview
import cv2
import time
import threading

app = Flask(__name__)
picam2 = Picamera2()

# Configure for video streaming
video_config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
picam2.configure(video_config)
picam2.start()

# Allow warm-up time
time.sleep(2)

frame_lock = threading.Lock()
latest_frame = None

def capture_frames():
    global latest_frame
    while True:
        frame = picam2.capture_array()
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with frame_lock:
            latest_frame = buffer.tobytes()

# Start capture thread
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

HTML_TEMPLATE = """
<html>
<head>
    <title>Live Pi Camera</title>
</head>
<body>
    <h1>Raspberry Pi Camera Stream</h1>
    <img src="{{ url_for('video_feed') }}" width="640" height="480">
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    def stream():
        while True:
            with frame_lock:
                if latest_frame:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.05)  # ~20 FPS
    return Response(stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
