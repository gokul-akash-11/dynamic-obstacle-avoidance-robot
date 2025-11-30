#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import json
import time
import math

class CVKalman2D:
    """A small Kalman filter wrapper using OpenCV's KalmanFilter for 2D constant velocity."""
    def __init__(self, cx: float, cy: float, dt: float = 0.1):
        # 4 states: x, y, vx, vy ; 2 measurements: x, y
        self.kf = cv2.KalmanFilter(4, 2)
        # Transition matrix
        self.kf.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # Measurement matrix H (2x4)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        # Covariances (tweakable)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5.0
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 1.0

        # Initialize statePost from measurement: [x, y, vx=0, vy=0]
        self.kf.statePost = np.array([[np.float32(cx)], [np.float32(cy)], [0.0], [0.0]], dtype=np.float32)

    def predict(self):
        """Returns predicted state (4x1) as numpy array."""
        p = self.kf.predict()
        return p  # shape (4,1)

    def correct(self, cx: float, cy: float):
        meas = np.array([[np.float32(cx)], [np.float32(cy)]], dtype=np.float32)
        self.kf.correct(meas)

    def predict_steps(self, steps=10):
        """Simulate forward using the F matrix to produce future (x,y) pairs without changing the filter state."""
        F = self.kf.transitionMatrix.astype(np.float64)
        temp = self.kf.statePost.astype(np.float64).copy()
        fut = []
        for _ in range(steps):
            temp = F.dot(temp)
            fut.append((float(temp[0,0]), float(temp[1,0])))
        return fut

class TrackerKFNode(Node):
    def __init__(self):
        super().__init__('tracker_kf_visual')
        self.bridge = CvBridge()
        # subscribe to detection JSON bbox centers
        self.sub_det = self.create_subscription(String, '/detected_objects_bbox', self.cb_det, 10)
        # subscribe to camera image for visualization
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.cb_img, 5)
        # publish predicted paths (optional)
        self.pub = self.create_publisher(String, '/predicted_paths', 10)

        self.get_logger().info("TrackerKF (visual) started - listening to /detected_objects_bbox & /camera/image_raw")

        # runtime buffers
        self.latest_img = None
        self.latest_dets = []  # list of dicts: {'label','cx','cy','w','h'}
        self.tracks = {}  # track_id -> {'kf': CVKalman2D, 'last_seen':timestamp, 'label':str}
        self.next_id = 1
        self.max_age = 1.0  # seconds before track removal
        self.match_thresh = 80.0  # pixels threshold for association
        # create an OpenCV window
        cv2.namedWindow('Tracker + Predictions', cv2.WINDOW_NORMAL)

    def cb_det(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            if isinstance(parsed, list):
                self.latest_dets = parsed
            else:
                # empty or other format -> clear
                self.latest_dets = []
        except Exception:
            self.get_logger().warn('Failed to parse detection JSON')
            self.latest_dets = []

    def cb_img(self, msg: Image):
        # convert image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        self.latest_img = frame.copy()
        now = time.time()

        # process detections and update tracks
        centroids = []
        for d in self.latest_dets:
            try:
                cx = float(d.get('cx', 0.0))
                cy = float(d.get('cy', 0.0))
                label = d.get('label', '')
                centroids.append({'cx': cx, 'cy': cy, 'label': label})
            except Exception:
                continue

        used = set()

        # Associate detections to existing tracks by nearest neighbor (simple)
        for c in centroids:
            best_tid = None
            best_dist = float('inf')
            for tid, info in self.tracks.items():
                if tid in used:
                    continue
                st = info['kf'].kf.statePost  # (4,1)
                sx = float(st[0,0]); sy = float(st[1,0])
                d2 = math.hypot(sx - c['cx'], sy - c['cy'])
                if d2 < best_dist:
                    best_dist = d2
                    best_tid = tid
            if best_tid is not None and best_dist < self.match_thresh:
                # update
                self.tracks[best_tid]['kf'].correct(c['cx'], c['cy'])
                self.tracks[best_tid]['last_seen'] = now
                self.tracks[best_tid]['label'] = c['label']
                used.add(best_tid)
            else:
                # new track, initialize KFilter with measurement to avoid zero-jump
                kf = CVKalman2D(c['cx'], c['cy'], dt=0.1)
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = {'kf': kf, 'last_seen': now, 'label': c['label']}
                used.add(tid)

        # clean old tracks
        to_remove = [tid for tid,info in self.tracks.items() if now - info['last_seen'] > self.max_age]
        for tid in to_remove:
            del self.tracks[tid]

        # visualize on frame
        vis = self.latest_img.copy()
        publish_dict = {}
        for tid, info in self.tracks.items():
            # predict current state (non-destructive)
            pred = info['kf'].predict()  # (4,1) float32
            px = float(pred[0,0]); py = float(pred[1,0])
            vx = float(pred[2,0]); vy = float(pred[3,0])
            label = info.get('label','')
            # draw current predicted center
            cv2.circle(vis, (int(round(px)), int(round(py))), 6, (0,255,0), -1)
            cv2.putText(vis, f"ID{tid}:{label}", (int(round(px))+6, int(round(py))-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            # compute future steps (10 steps by default)
            fut = info['kf'].predict_steps(steps=10)
            publish_dict[str(tid)] = {'label': label, 'future': [[float(a),float(b)] for (a,b) in fut]}
            # draw future points
            for (fx, fy) in fut:
                cv2.circle(vis, (int(round(fx)), int(round(fy))), 3, (0,0,255), -1)

        # publish predicted paths JSON
        try:
            out = String()
            out.data = json.dumps(publish_dict)
            self.pub.publish(out)
        except Exception:
            pass

        # show window
        cv2.imshow('Tracker + Predictions', vis)
        key = cv2.waitKey(1)
        if key == 27:
            # exit gracefully on ESC
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrackerKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
