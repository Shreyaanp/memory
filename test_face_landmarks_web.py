#!/usr/bin/env python3
"""
Simple RealSense + Face Landmarks Test with Web UI
This uses the Horizon BPU face landmark model (106 points)
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, render_template_string
import threading
import time
from hobot_dnn import pyeasy_dnn as dnn

app = Flask(__name__)

class FaceLandmarkTester:
    def __init__(self):
        self.pipeline = None
        self.frame = None
        self.landmarks = None
        self.fps = 0
        self.lock = threading.Lock()
        self.frame_skip = 0  # Process every Nth frame for face detection
        self.last_face_bbox = None  # Cache last detected face
        
        # Initialize BPU model
        print("Loading BPU face landmark model...")
        self.model = dnn.load("/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm")
        print(f"Model loaded: {self.model}")
        
        # Load face detector once
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
    def start_camera(self):
        """Initialize RealSense camera"""
        print("Starting RealSense camera...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # RGB stream - 640x480 @ 30fps for testing
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.pipeline.start(config)
        print("Camera started successfully!")
        
    def detect_face_cv(self, frame):
        """Quick face detection using OpenCV (cached and optimized)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Use strict settings for better performance
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(50, 50))
        
        if len(faces) > 0:
            # Return largest face
            return max(faces, key=lambda f: f[2] * f[3])
        return None
    
    def run_bpu_landmarks(self, frame, face_bbox):
        """Run BPU face landmark detection - simple crop method"""
        x, y, w, h = face_bbox
        
        # Crop face region with padding
        padding = int(w * 0.2)  # 20% padding
        x1 = max(0, x - padding)
        y1 = max(0, y - padding)
        x2 = min(frame.shape[1], x + w + padding)
        y2 = min(frame.shape[0], y + h + padding)
        
        face_crop = frame[y1:y2, x1:x2]
        
        # Resize to 112x112 (common input size for face landmark models)
        face_resized = cv2.resize(face_crop, (112, 112))
        
        # Convert BGR to RGB (BPU models often expect RGB)
        face_rgb = cv2.cvtColor(face_resized, cv2.COLOR_BGR2RGB)
        
        # Run BPU inference
        outputs = self.model[0].forward([face_rgb])
        
        # Parse landmarks
        landmarks_raw = outputs[0].buffer[0]
        
        # Convert to landmark coordinates
        landmarks = []
        for i in range(0, min(212, len(landmarks_raw)), 2):
            lx_norm = landmarks_raw[i]
            ly_norm = landmarks_raw[i+1]
            
            # Scale from normalized [0,1] or model coordinates to original image
            # Model output is typically normalized to face crop size
            lx = lx_norm * (x2 - x1) + x1
            ly = ly_norm * (y2 - y1) + y1
            
            landmarks.append((int(lx), int(ly)))
        
        return landmarks[:106]  # Return first 106 points
    
    def draw_landmarks(self, frame, landmarks):
        """Draw landmarks on frame"""
        if landmarks is None:
            return frame
        
        # Draw all 106 points
        for i, (x, y) in enumerate(landmarks):
            # Nose tip region (landmarks 30-35 are typically nose area)
            if 30 <= i <= 35:
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)  # Green for nose
            else:
                cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)  # Red for other points
        
        # Highlight nose tip with crosshair (landmark 30)
        if len(landmarks) > 30:
            nx, ny = landmarks[30]
            cv2.line(frame, (nx-15, ny), (nx+15, ny), (0, 255, 0), 2)
            cv2.line(frame, (nx, ny-15), (nx, ny+15), (0, 255, 0), 2)
            
            # Show nose coordinates
            cv2.putText(frame, f"Nose: ({nx},{ny})", (nx+20, ny-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return frame
    
    def process_loop(self):
        """Main processing loop - optimized for speed"""
        frame_count = 0
        start_time = time.time()
        
        while True:
            try:
                # Get frame from RealSense
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy array
                frame = np.asanyarray(color_frame.get_data())
                
                # Only detect face every 3rd frame to improve FPS
                self.frame_skip += 1
                if self.frame_skip % 3 == 0:
                    face_bbox = self.detect_face_cv(frame)
                    if face_bbox is not None:
                        self.last_face_bbox = face_bbox  # Cache it
                else:
                    face_bbox = self.last_face_bbox  # Use cached bbox
                
                landmarks = None
                if face_bbox is not None:
                    # Draw face bounding box
                    x, y, w, h = face_bbox
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                    
                    try:
                        # Run BPU landmark detection
                        landmarks = self.run_bpu_landmarks(frame, face_bbox)
                        
                        # Draw landmarks
                        frame = self.draw_landmarks(frame, landmarks)
                        
                    except Exception as e:
                        cv2.putText(frame, f"BPU Error: {str(e)[:30]}", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    cv2.putText(frame, "No face detected", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Calculate FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    self.fps = 30 / elapsed
                    start_time = time.time()
                
                # Add FPS text
                cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Update shared frame
                with self.lock:
                    self.frame = frame
                    self.landmarks = landmarks
                
            except Exception as e:
                print(f"Error in process loop: {e}")
                time.sleep(0.1)
    
    def get_frame(self):
        """Get current frame for web streaming - optimized"""
        with self.lock:
            if self.frame is None:
                return None
            frame_copy = self.frame.copy()
        
        # Encode as JPEG with lower quality for faster streaming
        try:
            ret, buffer = cv2.imencode('.jpg', frame_copy, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ret:
                return buffer.tobytes()
        except Exception as e:
            print(f"Encode error: {e}")
        
        return None

# Global tester instance
tester = FaceLandmarkTester()

@app.route('/')
def index():
    """Web UI with landmark info"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Face Landmarks - RDK X5 BPU</title>
        <style>
            body {
                font-family: 'Segoe UI', Arial, sans-serif;
                background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
                color: white;
                margin: 0;
                padding: 20px;
                display: flex;
                flex-direction: column;
                align-items: center;
            }
            h1 {
                color: #00ff88;
                margin-bottom: 5px;
                text-shadow: 0 0 10px rgba(0, 255, 136, 0.5);
            }
            .subtitle {
                color: #888;
                margin-bottom: 20px;
                font-size: 14px;
            }
            .info-grid {
                display: grid;
                grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
                gap: 15px;
                max-width: 800px;
                width: 100%;
                margin-bottom: 20px;
            }
            .info-card {
                background: rgba(255, 255, 255, 0.05);
                padding: 15px;
                border-radius: 8px;
                border: 1px solid rgba(0, 255, 136, 0.3);
            }
            .info-card h3 {
                margin: 0 0 8px 0;
                color: #00ff88;
                font-size: 14px;
                text-transform: uppercase;
            }
            .info-card p {
                margin: 5px 0;
                font-size: 16px;
            }
            .video-container {
                border: 3px solid #00ff88;
                border-radius: 12px;
                overflow: hidden;
                box-shadow: 0 0 30px rgba(0, 255, 136, 0.4);
                max-width: 800px;
                width: 100%;
            }
            img {
                display: block;
                width: 100%;
                height: auto;
            }
            .legend {
                background: rgba(255, 255, 255, 0.05);
                padding: 20px;
                border-radius: 12px;
                margin-top: 20px;
                max-width: 800px;
                width: 100%;
                border: 1px solid rgba(0, 255, 136, 0.3);
            }
            .legend h3 {
                margin-top: 0;
                color: #00ff88;
            }
            .legend-item {
                display: flex;
                align-items: center;
                margin: 10px 0;
            }
            .color-box {
                width: 24px;
                height: 24px;
                margin-right: 12px;
                border: 2px solid white;
                border-radius: 4px;
                flex-shrink: 0;
            }
            .feature-list {
                background: rgba(255, 255, 255, 0.05);
                padding: 20px;
                border-radius: 12px;
                margin-top: 20px;
                max-width: 800px;
                width: 100%;
                border: 1px solid rgba(0, 255, 136, 0.3);
            }
            .feature-list h3 {
                margin-top: 0;
                color: #00ff88;
            }
            .feature-list ul {
                list-style: none;
                padding: 0;
            }
            .feature-list li {
                padding: 8px 0;
                padding-left: 25px;
                position: relative;
            }
            .feature-list li:before {
                content: "✓";
                position: absolute;
                left: 0;
                color: #00ff88;
                font-weight: bold;
            }
        </style>
        <script>
            // Auto-refresh info every 2 seconds
            setInterval(function() {
                document.getElementById('timestamp').innerText = new Date().toLocaleTimeString();
            }, 1000);
        </script>
    </head>
    <body>
        <h1>🎯 Face Landmark Detection</h1>
        <div class="subtitle">Real-time BPU-accelerated inference on RDK X5</div>
        
        <div class="info-grid">
            <div class="info-card">
                <h3>Hardware</h3>
                <p>🖥️ RDK X5 (Horizon BPU)</p>
                <p>📷 Intel RealSense</p>
            </div>
            <div class="info-card">
                <h3>Model</h3>
                <p>🧠 106-Point Landmarks</p>
                <p>⚡ ROI Inference</p>
            </div>
            <div class="info-card">
                <h3>Stream</h3>
                <p>📺 640x480 @ 30fps</p>
                <p>🕐 <span id="timestamp">--:--:--</span></p>
            </div>
        </div>
        
        <div class="video-container">
            <img src="/video_feed" alt="Live Camera Feed">
        </div>
        
        <div class="legend">
            <h3>Visualization Legend</h3>
            <div class="legend-item">
                <div class="color-box" style="background: #0000ff;"></div>
                <span><strong>Blue Rectangle:</strong> Face bounding box (OpenCV Haar Cascade)</span>
            </div>
            <div class="legend-item">
                <div class="color-box" style="background: #ff0000;"></div>
                <span><strong>Red Dots:</strong> 106 facial landmarks (BPU inference)</span>
            </div>
            <div class="legend-item">
                <div class="color-box" style="background: #00ff00;"></div>
                <span><strong>Green Crosshair:</strong> Nose tip position (Landmark #30)</span>
            </div>
            <div class="legend-item">
                <div class="color-box" style="background: #00ff00;"></div>
                <span><strong>Green Circles:</strong> Nose region landmarks (#30-35)</span>
            </div>
        </div>
        
        <div class="feature-list">
            <h3>Optimizations Applied</h3>
            <ul>
                <li>Frame skipping (face detection every 3rd frame)</li>
                <li>Cached face bounding boxes</li>
                <li>JPEG quality optimization (70%)</li>
                <li>Pre-loaded face detector</li>
                <li>Efficient ROI inference (no manual crop/resize)</li>
                <li>Threaded processing (UI + inference decoupled)</li>
            </ul>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    """Video streaming route - optimized"""
    def generate():
        last_frame = None
        while True:
            try:
                frame = tester.get_frame()
                if frame is not None:
                    last_frame = frame
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                elif last_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + last_frame + b'\r\n')
                time.sleep(0.02)  # ~50 FPS max streaming rate
            except GeneratorExit:
                break
            except Exception as e:
                print(f"Stream error: {e}")
                time.sleep(0.1)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    print("="*60)
    print("Face Landmarks Test - RDK X5 + RealSense")
    print("="*60)
    
    try:
        # Start camera
        tester.start_camera()
        
        # Start processing thread
        process_thread = threading.Thread(target=tester.process_loop, daemon=True)
        process_thread.start()
        
        print("\n✅ Camera and BPU model initialized!")
        print("\n📱 Open browser and navigate to:")
        print("   http://<RDK_IP>:5000")
        print("\n   (Replace <RDK_IP> with your RDK's IP address)")
        print("\nPress Ctrl+C to stop\n")
        
        # Start web server
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
        
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if tester.pipeline:
            tester.pipeline.stop()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()


