"""Colour-based crop detection on an aligned RGB-D stream.

Works with the Gazebo camera and with a RealSense D4xx (realsense2_camera,
align_depth enabled). Each class is a set of HSV ranges plus a blob-size
window; the 3D position is taken from the median depth inside the blob and
pushed back by the object radius, so it is the object centre, not its surface.

Colour thresholds are the simplest detector that works for the simulated
crops. For real fields, replace `segment()` with a trained model (e.g. a
YOLO segmentation network) and keep the rest of the pipeline.
"""
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
from geometry_msgs.msg import Point
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class ColorClass:
    name: str
    hsv_ranges: list          # [(h_lo, s_lo, v_lo, h_hi, s_hi, v_hi), ...] OpenCV scale (H 0-179)
    min_area: int = 30        # px
    max_area: int = 20000     # px
    radius: float = 0.01      # m, surface -> centre correction
    color: tuple = (0, 0, 255)


@dataclass
class Detection:
    label: str
    pixel: tuple
    area: int
    position: np.ndarray                  # object centre in the target frame
    depth: float
    extent: float                         # approximate blob diameter (m)
    extra: dict = field(default_factory=dict)


def image_to_array(msg):
    """sensor_msgs/Image -> numpy (no cv_bridge, which breaks with NumPy 2)."""
    if msg.encoding in ('rgb8', 'bgr8'):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width].copy()
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return raw.astype(np.float32) / 1000.0   # RealSense depth is in mm
    raise ValueError(f'unsupported image encoding {msg.encoding}')


def array_to_image(img, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg


def segment(bgr, cls):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for h0, s0, v0, h1, s1, v1 in cls.hsv_ranges:
        mask |= cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


class CropDetector:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        p = node.declare_parameter
        self.color_topic = p('camera.color_topic', '/camera/color/image_raw').value
        self.depth_topic = p('camera.depth_topic', '/camera/aligned_depth_to_color/image_raw').value
        self.info_topic = p('camera.info_topic', '/camera/color/camera_info').value
        self.camera_frame = p('camera.frame', 'camera_color_optical_frame').value
        # Only detections inside this box (target frame) count, e.g. the crop
        # row, so fruit already in the crate is not harvested twice.
        self.region = p('detection_region', [-1e3, 1e3, -1e3, 1e3, -1e3, 1e3]).value
        self._lock = threading.Lock()
        self._color = self._depth = self._info = None
        node.create_subscription(Image, self.color_topic, self._on_color, 2)
        node.create_subscription(Image, self.depth_topic, self._on_depth, 2)
        node.create_subscription(CameraInfo, self.info_topic, self._on_info, 2)
        self.debug_pub = node.create_publisher(Image, '/agrobot/detections/image', 2)
        self.marker_pub = node.create_publisher(MarkerArray, '/agrobot/detections/markers', 2)

    def _on_color(self, msg):
        with self._lock:
            self._color = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_info(self, msg):
        with self._lock:
            self._info = msg

    def wait_for_camera(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._color and self._depth and self._info:
                    return True
            time.sleep(0.1)
        return False

    def fresh_frame(self, settle=0.6, timeout=5.0):
        """Wait until a frame newer than now + settle (robot at rest) arrives."""
        time.sleep(settle)
        t0 = self.node.get_clock().now()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                c, d, i = self._color, self._depth, self._info
            if c and d and Time.from_msg(c.header.stamp) >= t0 and Time.from_msg(d.header.stamp) >= t0:
                return image_to_array(c), image_to_array(d), i, c.header.stamp
            time.sleep(0.05)
        raise RuntimeError('no fresh camera frame')

    def detect(self, classes, target_frame='world', annotate=True):
        bgr, depth, info, stamp = self.fresh_frame()
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        cam_to_target = self.robot.lookup(target_frame, self.camera_frame)
        debug = bgr.copy() if annotate else None
        detections = []
        for cls in classes:
            mask = segment(bgr, cls)
            n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
            for i in range(1, n):
                area = int(stats[i, cv2.CC_STAT_AREA])
                if not cls.min_area <= area <= cls.max_area:
                    continue
                d = depth[labels == i]
                d = d[np.isfinite(d) & (d > 0.05)]
                if d.size < 5:
                    continue
                z = float(np.median(d))
                u, v = centroids[i]
                ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                surface = ray * z
                centre = surface + ray / np.linalg.norm(ray) * cls.radius
                pos = (cam_to_target @ np.append(centre, 1.0))[:3]
                r = self.region
                if not (r[0] <= pos[0] <= r[1] and r[2] <= pos[1] <= r[3] and r[4] <= pos[2] <= r[5]):
                    continue
                extent = 2.0 * np.sqrt(area / np.pi) * z / fx
                detections.append(Detection(cls.name, (float(u), float(v)), area, pos, z, extent))
                if annotate:
                    x, y, w, h = stats[i, :4]
                    cv2.rectangle(debug, (x - 2, y - 2), (x + w + 2, y + h + 2), cls.color, 2)
                    cv2.putText(debug, cls.name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, cls.color, 1)
        if annotate:
            self.debug_pub.publish(array_to_image(debug, stamp, self.camera_frame))
            self.publish_markers(detections, target_frame)
        return detections, debug

    def publish_markers(self, detections, frame):
        msg = MarkerArray()
        msg.markers.append(Marker(action=Marker.DELETEALL))
        for k, det in enumerate(detections):
            m = Marker()
            m.header.frame_id = frame
            m.ns = det.label
            m.id = k
            m.type = Marker.SPHERE
            m.pose.position = Point(x=float(det.position[0]), y=float(det.position[1]), z=float(det.position[2]))
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = max(0.01, det.extent)
            m.color.a = 0.8
            m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            msg.markers.append(m)
        self.marker_pub.publish(msg)


def merge_detections(detections, radius=0.02):
    """Fuse detections of the same object seen from several stations."""
    merged = []
    for det in detections:
        for m in merged:
            if m['label'] == det.label and np.linalg.norm(m['position'] - det.position) < radius:
                m['samples'].append(det.position)
                m['position'] = np.mean(m['samples'], axis=0)
                m['extent'] = max(m['extent'], det.extent)
                break
        else:
            merged.append({'label': det.label, 'position': det.position.copy(),
                           'samples': [det.position], 'extent': det.extent})
    return merged


def classes_from_params(node, prefix, names):
    """Reads ColorClass definitions from parameters <prefix>.<name>.*"""
    out = []
    for name in names:
        p = f'{prefix}.{name}'
        ranges = node.declare_parameter(f'{p}.hsv_ranges', [0, 0, 0, 0, 0, 0]).value
        out.append(ColorClass(
            name=name,
            hsv_ranges=[tuple(ranges[i:i + 6]) for i in range(0, len(ranges), 6)],
            min_area=node.declare_parameter(f'{p}.min_area', 30).value,
            max_area=node.declare_parameter(f'{p}.max_area', 20000).value,
            radius=node.declare_parameter(f'{p}.radius', 0.01).value,
            color=tuple(node.declare_parameter(f'{p}.draw_bgr', [0, 0, 255]).value)))
    return out
