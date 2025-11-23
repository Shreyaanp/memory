#!/usr/bin/env python3
"""
Nose tracking using MediaPipe Face Mesh on CPU
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from flask import Flask, Response, render_template_string
import threading
import time
import mediapipe as mp

app = Flask(__name__)

class MediaPipeNoseTracker:
    def __init__(self):
        self.pipeline = None
        self.frame = None
        self.nose_pos = None
        self.fps = 0
        self.lock = threading.Lock()
        
        # Nose tracking history for smoothing
        self.nose_history = []
        self.history_size = 5
        
        # Performance tracking
        self.detection_count = 0
        self.no_detection_count = 0
        self.total_inference_time = 0
        
        # Initialize MediaPipe Face Mesh
        print("Initializing MediaPipe Face Mesh (CPU)...")
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,  # Enable for better accuracy
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            static_image_mode=False  # Video mode for better tracking
        )
        print("MediaPipe initialized with refined landmarks!")
        
    def smooth_nose_position(self, new_pos):
        """Apply temporal smoothing to nose position"""
        self.nose_history.append(new_pos)
        if len(self.nose_history) > self.history_size:
            self.nose_history.pop(0)
        
        # Average over history
        if len(self.nose_history) > 0:
            avg_x = sum(pos[0] for pos in self.nose_history) / len(self.nose_history)
            avg_y = sum(pos[1] for pos in self.nose_history) / len(self.nose_history)
            return (int(avg_x), int(avg_y))
        return new_pos
    
    def start_camera(self):
        """Initialize RealSense camera"""
        print("Starting RealSense camera...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        print("Camera started successfully!")
        
    def process_loop(self):
        """Main processing loop with detailed logging"""
        frame_count = 0
        start_time = time.time()
        last_log_time = time.time()
        
        print("\n" + "="*60)
        print("Starting processing loop...")
        print("="*60 + "\n")
        
        while True:
            try:
                # Get frame from RealSense
                t_capture_start = time.time()
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                t_capture = (time.time() - t_capture_start) * 1000
                
                if not color_frame:
                    print(f"[WARNING] Frame {frame_count}: No color frame received")
                    continue
                
                # Convert to numpy array
                t_convert_start = time.time()
                frame = np.asanyarray(color_frame.get_data())
                h, w, _ = frame.shape
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                t_convert = (time.time() - t_convert_start) * 1000
                
                # Process with MediaPipe
                t_inference_start = time.time()
                results = self.face_mesh.process(rgb_frame)
                t_inference = (time.time() - t_inference_start) * 1000
                self.total_inference_time += t_inference
                
                nose_pos = None
                if results.multi_face_landmarks:
                    self.detection_count += 1
                    face_landmarks = results.multi_face_landmarks[0]
                    
                    # Get nose tip (landmark 1) and surrounding points for stability
                    nose_tip = face_landmarks.landmark[1]
                    nose_bridge = face_landmarks.landmark[168]
                    
                    # Use nose tip as primary
                    nose_x_raw = int(nose_tip.x * w)
                    nose_y_raw = int(nose_tip.y * h)
                    
                    # Apply smoothing
                    nose_pos_raw = (nose_x_raw, nose_y_raw)
                    nose_pos = self.smooth_nose_position(nose_pos_raw)
                    nose_x, nose_y = nose_pos
                    
                    # Draw a subset of landmarks for visualization (every 5th point for better visibility)
                    t_draw_start = time.time()
                    for idx in range(0, len(face_landmarks.landmark), 5):
                        landmark = face_landmarks.landmark[idx]
                        x = int(landmark.x * w)
                        y = int(landmark.y * h)
                        cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)
                    
                    # Draw nose bridge point (blue)
                    bridge_x = int(nose_bridge.x * w)
                    bridge_y = int(nose_bridge.y * h)
                    cv2.circle(frame, (bridge_x, bridge_y), 5, (255, 0, 0), -1)
                    
                    # Draw nose tip prominently (green)
                    cv2.circle(frame, (nose_x, nose_y), 10, (0, 255, 0), -1)
                    cv2.line(frame, (nose_x-25, nose_y), (nose_x+25, nose_y), (0, 255, 0), 3)
                    cv2.line(frame, (nose_x, nose_y-25), (nose_x, nose_y+25), (0, 255, 0), 3)
                    
                    # Draw smoothing indicator
                    cv2.line(frame, (nose_x_raw, nose_y_raw), (nose_x, nose_y), (255, 255, 0), 2)
                    
                    t_draw = (time.time() - t_draw_start) * 1000
                    
                    # Show coordinates
                    cv2.putText(frame, f"Nose: ({nose_x},{nose_y})", (nose_x+30, nose_y-15), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Normalize to [-1, 1] for circular motion tracking
                    center_x, center_y = w // 2, h // 2
                    norm_x = (nose_x - center_x) / (w // 2)
                    norm_y = (nose_y - center_y) / (h // 2)
                    
                    cv2.putText(frame, f"Norm: ({norm_x:.3f}, {norm_y:.3f})", (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Show timing breakdown
                    cv2.putText(frame, f"Capture: {t_capture:.1f}ms | Convert: {t_convert:.1f}ms", (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    cv2.putText(frame, f"Inference: {t_inference:.1f}ms | Draw: {t_draw:.1f}ms", (10, 140), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    
                    # Log detailed info every 5 seconds
                    if time.time() - last_log_time > 5.0:
                        avg_inference = self.total_inference_time / (self.detection_count or 1)
                        detection_rate = self.detection_count / (self.detection_count + self.no_detection_count) * 100
                        print(f"[STATS] Frame {frame_count}:")
                        print(f"  - Detection rate: {detection_rate:.1f}%")
                        print(f"  - Avg inference: {avg_inference:.1f}ms")
                        print(f"  - Current FPS: {self.fps:.1f}")
                        print(f"  - Nose history size: {len(self.nose_history)}")
                        print(f"  - Raw position: ({nose_x_raw}, {nose_y_raw})")
                        print(f"  - Smoothed position: ({nose_x}, {nose_y})")
                        last_log_time = time.time()
                else:
                    self.no_detection_count += 1
                    cv2.putText(frame, "No face detected", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.putText(frame, f"Inference: {t_inference:.1f}ms (no detection)", (10, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    
                    # Log no detection occasionally
                    if self.no_detection_count % 30 == 1:
                        print(f"[WARNING] Frame {frame_count}: No face landmarks detected (inference took {t_inference:.1f}ms)")
                
                # Calculate FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    self.fps = 30 / elapsed
                    start_time = time.time()
                
                # Add FPS text
                cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                
                # Draw center crosshair
                center_x, center_y = w // 2, h // 2
                cv2.circle(frame, (center_x, center_y), 5, (128, 128, 128), 2)
                cv2.line(frame, (center_x-40, center_y), (center_x+40, center_y), (128, 128, 128), 1)
                cv2.line(frame, (center_x, center_y-40), (center_x, center_y+40), (128, 128, 128), 1)
                
                # Update shared frame
                with self.lock:
                    self.frame = frame
                    self.nose_pos = nose_pos
                
            except Exception as e:
                print(f"\n[ERROR] Frame {frame_count}: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(0.1)
    
    def get_frame(self):
        """Get current frame for web streaming"""
        with self.lock:
            if self.frame is None:
                return None
            frame_copy = self.frame.copy()
        
        # Encode as JPEG
        try:
            ret, buffer = cv2.imencode('.jpg', frame_copy, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ret:
                return buffer.tobytes()
        except Exception as e:
            print(f"Encode error: {e}")
        
        return None

# Global tracker instance
tracker = MediaPipeNoseTracker()

@app.route('/')
def index():
    """Web UI"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Nose Tracking - MediaPipe</title>
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
            .info {
                background: rgba(255, 255, 255, 0.05);
                padding: 20px;
                border-radius: 12px;
                margin-top: 20px;
                max-width: 800px;
                width: 100%;
                border: 1px solid rgba(0, 255, 136, 0.3);
            }
            .info h3 {
                margin-top: 0;
                color: #00ff88;
            }
            .warning {
                background: rgba(255, 165, 0, 0.1);
                border: 1px solid rgba(255, 165, 0, 0.5);
                padding: 15px;
                border-radius: 8px;
                margin-top: 15px;
            }
        </style>
    </head>
    <body>
        <h1>🎯 Nose Position Tracking</h1>
        <div class="subtitle">MediaPipe Face Mesh (468 landmarks, CPU)</div>
        
        <div class="video-container">
            <img src="/video_feed" alt="Live Camera Feed">
        </div>
        
        <div class="info">
            <h3>MediaPipe Face Mesh:</h3>
            <p><strong>Red Dots:</strong> Facial landmarks (every 5th point shown for clarity)</p>
            <p><strong>Blue Circle:</strong> Nose bridge reference point (Landmark #168)</p>
            <p><strong>Green Crosshair:</strong> Smoothed nose tip position (Landmark #1)</p>
            <p><strong>Yellow Line:</strong> Smoothing adjustment (raw → smoothed)</p>
            <p><strong>Gray Crosshair + Circle:</strong> Screen center reference</p>
            <p><strong>Timing Info:</strong> Breakdown of processing time per operation</p>
            
            <div class="warning">
                ⚠️ <strong>Performance Note:</strong> MediaPipe on CPU will be slower than OpenCV.
                Expected FPS: 5-15 fps depending on CPU load.
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html)

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    def generate():
        last_frame = None
        while True:
            try:
                frame = tracker.get_frame()
                if frame is not None:
                    last_frame = frame
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                elif last_frame is not None:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + last_frame + b'\r\n')
                time.sleep(0.05)  # ~20 FPS max streaming rate
            except GeneratorExit:
                break
            except Exception as e:
                print(f"Stream error: {e}")
                time.sleep(0.1)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    print("="*60)
    print("MediaPipe Nose Tracking - RDK X5 + RealSense")
    print("="*60)
    
    try:
        # Start camera
        tracker.start_camera()
        
        # Start processing thread
        process_thread = threading.Thread(target=tracker.process_loop, daemon=True)
        process_thread.start()
        
        print("\n✅ Camera and MediaPipe initialized!")
        print("\n📱 Open browser: http://172.21.183.230:5000")
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
        if tracker.pipeline:
            tracker.pipeline.stop()
        tracker.face_mesh.close()
        print("Cleanup complete.")

if __name__ == "__main__":
    main()

