#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import threading
import json
import numpy as np

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # Subscribe to camera image
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        # Publish JSON list of detections with bbox centers
        self.publisher_ = self.create_publisher(String, '/detected_objects_bbox', 10)

        self.bridge = CvBridge()
        # small model for real-time demo
        self.model = YOLO('yolov8n.pt')

        self.get_logger().info('YOLOv8 detector node started')

        # Shared frame buffer + lock to avoid flicker and blocking
        self.latest_frame = None
        self.lock = threading.Lock()

        # Start visualization thread (daemon so it exits with node)
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

    def image_callback(self, msg: Image):
        # Convert ROS Image to OpenCV frame and store in buffer
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        with self.lock:
            # keep the latest frame only
            self.latest_frame = frame

    def display_loop(self):
        cv2.namedWindow("YOLOv8 Detection", cv2.WINDOW_NORMAL)
        # tune inference settings here if needed
        infer_kwargs = {"imgsz": 640, "conf": 0.35}

        while rclpy.ok():
            frame = None
            with self.lock:
                if self.latest_frame is not None:
                    # copy to avoid modifying shared buffer
                    frame = self.latest_frame.copy()

            if frame is None:
                # small sleep to reduce CPU when idle
                cv2.waitKey(10)
                continue

            # Run inference (Ultralytics YOLO returns a Results object)
            try:
                results = self.model(frame, **infer_kwargs)
            except Exception as e:
                # Log inference errors and continue
                self.get_logger().error(f"YOLO inference error: {e}")
                cv2.waitKey(1)
                continue

            # Visualize annotated frame (if any)
            try:
                annotated = results[0].plot()
            except Exception:
                annotated = frame

            # Extract detections -> publish as JSON with bbox centers
            detections = []
            try:
                boxes = results[0].boxes  # Boxes object (may be empty)
                # boxes.xyxy and boxes.cls handling: robust to tensor/array
                for b in boxes:
                    # xyxy may be tensor or numpy
                    try:
                        xyxy = b.xyxy.cpu().numpy().tolist()[0] if hasattr(b.xyxy, 'cpu') else b.xyxy.tolist()
                    except Exception:
                        # fallback if xyxy is a simple list/array
                        xyxy = list(b.xyxy)
                    x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
                    w = x2 - x1
                    h = y2 - y1
                    cx = x1 + w / 2.0
                    cy = y1 + h / 2.0
                    # class id handling
                    try:
                        cls_id = int(b.cls.cpu().numpy().item()) if hasattr(b.cls, 'cpu') else int(b.cls)
                    except Exception:
                        # fallback if cls is simple scalar
                        cls_id = int(b.cls)
                    label = self.model.names.get(cls_id, str(cls_id)) if isinstance(self.model.names, dict) else self.model.names[cls_id]
                    detections.append({
                        "label": label,
                        "cx": float(cx),
                        "cy": float(cy),
                        "w": float(w),
                        "h": float(h)
                    })
            except Exception as e:
                # If structure unexpected, log and continue
                self.get_logger().warn(f"Error parsing boxes: {e}")

            # Publish JSON only if we have detections (optional: publish empty list too)
            out_msg = String()
            out_msg.data = json.dumps(detections)
            self.publisher_.publish(out_msg)

            # Small info log but avoid flooding the console
            if detections:
                # Log compact sample (first 3 labels)
                labels_sample = sorted({d["label"] for d in detections})
                self.get_logger().debug(f"Detections: {labels_sample}")

            # Display annotated frame
            cv2.imshow("YOLOv8 Detection", annotated)
            key = cv2.waitKey(1)
            if key == 27:  # ESC to close window
                break

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
