import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


CLASS_SPECS = {
    "person": {
        "id": 0,
        "keys": ("person", "male", "female"),
        "color": (0, 220, 0),
    },
    "traffic_light": {
        "id": 1,
        "keys": ("traffic_light", "traffic", "semaphore"),
        "color": (0, 180, 255),
    },
    "apriltag": {
        "id": 2,
        "keys": ("apriltag", "april_tag", "april-tag", "marker", "target_marker", "tag"),
        "color": (255, 120, 0),
    },
}

OBJECT_PROXY_SIZES = {
    "person": {
        "width": 0.55,
        "depth": 0.35,
        "height": 1.75,
        "z_offset": 0.875,
    },
    "traffic_light": {
        "width": 0.45,
        "depth": 0.25,
        "height": 1.20,
        "z_offset": 1.20,
    },
    "apriltag": {
        "width": 0.40,
        "depth": 0.02,
        "height": 0.40,
        "z_offset": 0.20,
    },
}

TARGET_MODES = {
    "around_people": "person",
    "around_traffic_lights": "traffic_light",
    "around_apriltags": "apriltag",
}

DESIRED_DISTRIBUTION = {
    "around_people": 2000,
    "around_traffic_lights": 1500,
    "around_apriltags": 1500,
    "background_random": 1000,
}

BACKGROUND_ANCHOR_KEYS = ("road", "crosswalk", "sidewalk", "spawn")
OBSTACLE_KEYS = (
    "house",
    "building",
    "shop",
    "cafe",
    "wall",
    "fence",
    "car",
    "suv",
    "person",
    "male",
    "female",
    "traffic_light",
    "traffic",
    "apriltag",
    "april_tag",
    "marker",
    "tag",
)
LINE_OF_SIGHT_BLOCKER_KEYS = (
    "house",
    "building",
    "shop",
    "cafe",
    "wall",
    "fence",
    "car",
    "suv",
)
NON_OBSTACLE_KEYS = ("ground", "road", "crosswalk", "sidewalk", "spawn")


@dataclass(frozen=True)
class Pose6D:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class WorldObject:
    name: str
    pose: Pose6D
    size_x: float
    size_y: float
    size_z: float
    yaw: float
    is_static: bool
    class_name: str | None = None
    is_obstacle: bool = False
    is_los_blocker: bool = False


@dataclass(frozen=True)
class PlannedShot:
    pose: Pose6D
    mode: str
    target: WorldObject | None
    attempts: int
    notes: str


@dataclass(frozen=True)
class YoloLabel:
    class_id: int
    class_name: str
    object_name: str
    x_center: float
    y_center: float
    width: float
    height: float
    distance: float
    width_px: float
    height_px: float
    score: float


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    horizontal_fov: float
    vertical_fov: float
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class ProjectionRejection:
    object_name: str
    class_name: str
    distance: float
    reason: str


@dataclass(frozen=True)
class LabelResult:
    labels: list[YoloLabel]
    rejections: list[ProjectionRejection]
    num_candidate_objects: int
    num_projected_objects: int
    target_was_labeled: bool


def _strip_namespace(tag):
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def _direct_child(element, child_name):
    for child in element:
        if _strip_namespace(child.tag) == child_name:
            return child
    return None


def _iter_named(element, child_name):
    for child in element.iter():
        if _strip_namespace(child.tag) == child_name:
            yield child


def parse_pose_text(text):
    values = [float(item) for item in (text or "").split()]
    values = (values + [0.0] * 6)[:6]
    return Pose6D(*values)


def read_pose(element):
    pose_element = _direct_child(element, "pose")
    if pose_element is None:
        return Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return parse_pose_text(pose_element.text)


def read_text_child(element, child_name):
    child = _direct_child(element, child_name)
    if child is None or child.text is None:
        return ""
    return child.text.strip()


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rotate_xy(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw


def rotation_matrix_from_euler(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def mat_vec_mul(matrix, vector):
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z,
        matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z,
        matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z,
    )


def mat_transpose_vec_mul(matrix, vector):
    x, y, z = vector
    return (
        matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z,
        matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z,
        matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z,
    )


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def default_world_path():
    candidates = [
        Path.cwd() / "src" / "gazebo_sim" / "world" / "city_cv.sdf",
        Path.cwd() / "gazebo_sim" / "world" / "city_cv.sdf",
        Path(__file__).resolve().parents[3] / "gazebo_sim" / "world" / "city_cv.sdf",
        Path(__file__).resolve().parents[4] / "src" / "gazebo_sim" / "world" / "city_cv.sdf",
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    if get_package_share_directory is not None:
        try:
            return str(Path(get_package_share_directory("gazebo_sim")) / "world" / "city_cv.sdf")
        except Exception:
            pass
    return str(Path("src") / "gazebo_sim" / "world" / "city_cv.sdf")


def default_output_dir():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "gazebo_sim" / "world" / "city_cv.sdf").exists():
            return str(parent / "datasets" / "city_cv_yolo_raw")
    return str(Path.cwd() / "datasets" / "city_cv_yolo_raw")


def find_world_element(root):
    if _strip_namespace(root.tag) == "world":
        return root
    for child in root:
        if _strip_namespace(child.tag) == "world":
            return child
    return None


def read_world_name(world_path):
    try:
        root = ET.parse(world_path).getroot()
    except (OSError, ET.ParseError):
        return "city_second"
    world = find_world_element(root)
    if world is None:
        return "city_second"
    return world.attrib.get("name", "city_second")


def classify_model(name):
    lower = name.lower()
    if any(key in lower for key in CLASS_SPECS["person"]["keys"]):
        return "person"
    if "lamp" not in lower and "pole" not in lower:
        if any(key in lower for key in CLASS_SPECS["traffic_light"]["keys"]):
            return "traffic_light"
        if "light" in lower and ("traffic" in lower or "semaphore" in lower):
            return "traffic_light"
    if any(key in lower for key in CLASS_SPECS["apriltag"]["keys"]):
        return "apriltag"
    return None


def fallback_size(name):
    lower = name.lower()
    if any(key in lower for key in ("house", "building", "shop", "cafe")):
        return 4.0, 4.0
    if any(key in lower for key in ("wall", "fence")):
        return 1.0, 0.3
    if any(key in lower for key in ("car", "suv")):
        return 3.0, 1.8
    if any(key in lower for key in ("person", "male", "female")):
        return 0.8, 0.8
    if any(key in lower for key in ("traffic_light", "traffic", "semaphore")):
        return 0.8, 0.8
    if any(key in lower for key in ("apriltag", "april_tag", "marker", "tag")):
        return 0.6, 0.6
    if "tree" in lower:
        return 1.0, 1.0
    return 1.0, 1.0


def fallback_size_z(name):
    class_name = classify_model(name)
    if class_name is not None:
        return OBJECT_PROXY_SIZES[class_name]["height"]
    lower = name.lower()
    if any(key in lower for key in ("house", "building", "shop", "cafe")):
        return 4.0
    if any(key in lower for key in ("wall", "fence")):
        return 1.5
    if any(key in lower for key in ("car", "suv")):
        return 1.6
    return 1.0


def geometry_extents(model):
    best = None
    best_area = 0.0
    for geometry in _iter_named(model, "geometry"):
        box = _direct_child(geometry, "box")
        if box is not None:
            size_text = read_text_child(box, "size")
            values = [float(item) for item in size_text.split()] if size_text else []
            if len(values) >= 3:
                sx, sy, sz = abs(values[0]), abs(values[1]), abs(values[2])
                area = sx * sy
                if area > best_area:
                    best = sx, sy, sz
                    best_area = area

        cylinder = _direct_child(geometry, "cylinder")
        if cylinder is not None:
            radius_text = read_text_child(cylinder, "radius")
            length_text = read_text_child(cylinder, "length")
            if radius_text:
                diameter = abs(float(radius_text)) * 2.0
                length = abs(float(length_text)) if length_text else diameter
                area = diameter * diameter
                if area > best_area:
                    best = diameter, diameter, length
                    best_area = area
    return best


def should_be_obstacle(name, size_x, size_y, is_static, class_name):
    lower = name.lower()
    if any(key in lower for key in NON_OBSTACLE_KEYS):
        return False
    if class_name is not None:
        return True
    if any(key in lower for key in OBSTACLE_KEYS):
        return True
    return is_static and max(size_x, size_y) >= 1.2


def load_world_objects(world_path):
    try:
        root = ET.parse(world_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f"Failed to parse SDF world '{world_path}': {exc}") from exc

    world = find_world_element(root)
    if world is None:
        raise RuntimeError(f"No <world> element found in SDF '{world_path}'.")

    objects = []
    for child in world:
        tag = _strip_namespace(child.tag)
        if tag == "model":
            name = child.attrib.get("name", "").strip()
            if not name:
                continue
            pose = read_pose(child)
            is_static = read_text_child(child, "static").lower() == "true"
            extents = geometry_extents(child)
        elif tag == "include":
            name = read_text_child(child, "name")
            if not name:
                uri = read_text_child(child, "uri")
                name = Path(uri.rstrip("/")).name if uri else "include"
            pose = read_pose(child)
            is_static = True
            extents = None
        else:
            continue

        class_name = classify_model(name)
        if extents is None:
            size_x, size_y = fallback_size(name)
            size_z = fallback_size_z(name)
        else:
            size_x, size_y, size_z = extents
        is_obstacle = should_be_obstacle(name, size_x, size_y, is_static, class_name)
        is_los_blocker = any(key in name.lower() for key in LINE_OF_SIGHT_BLOCKER_KEYS)
        objects.append(
            WorldObject(
                name=name,
                pose=pose,
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                yaw=pose.yaw,
                is_static=is_static,
                class_name=class_name,
                is_obstacle=is_obstacle,
                is_los_blocker=is_los_blocker,
            )
        )
    return objects


def categorize_objects(objects):
    targets = {class_name: [] for class_name in CLASS_SPECS}
    background_anchors = []
    obstacles = []
    los_blockers = []

    for obj in objects:
        if obj.class_name is not None:
            targets[obj.class_name].append(obj)
        if any(key in obj.name.lower() for key in BACKGROUND_ANCHOR_KEYS):
            background_anchors.append(obj)
        if obj.is_obstacle:
            obstacles.append(obj)
        if obj.is_los_blocker:
            los_blockers.append(obj)

    return targets, background_anchors, obstacles, los_blockers


def allocate_counts(num_images, targets, logger):
    available_weights = {}
    for mode, desired in DESIRED_DISTRIBUTION.items():
        class_name = TARGET_MODES.get(mode)
        if class_name is None or targets.get(class_name):
            available_weights[mode] = desired
        else:
            logger.warning(f"No SDF objects found for '{class_name}', redistributing {mode}.")

    if not available_weights:
        available_weights["background_random"] = num_images

    weight_sum = float(sum(available_weights.values()))
    raw_counts = {
        mode: (num_images * weight / weight_sum)
        for mode, weight in available_weights.items()
    }
    counts = {mode: int(math.floor(value)) for mode, value in raw_counts.items()}
    remaining = num_images - sum(counts.values())
    by_fraction = sorted(
        raw_counts,
        key=lambda mode: raw_counts[mode] - counts[mode],
        reverse=True,
    )
    for mode in by_fraction[:remaining]:
        counts[mode] += 1

    for mode in DESIRED_DISTRIBUTION:
        counts.setdefault(mode, 0)
    return counts


def point_to_footprint_distance(x, y, obj):
    local_x, local_y = rotate_xy(x - obj.pose.x, y - obj.pose.y, -obj.yaw)
    dx = abs(local_x) - obj.size_x * 0.5
    dy = abs(local_y) - obj.size_y * 0.5
    outside_dx = max(dx, 0.0)
    outside_dy = max(dy, 0.0)
    outside_distance = math.hypot(outside_dx, outside_dy)
    if dx <= 0.0 and dy <= 0.0:
        return -min(-dx, -dy)
    return outside_distance


def point_is_collision_free(x, y, obstacles, safety_margin):
    for obstacle in obstacles:
        if point_to_footprint_distance(x, y, obstacle) <= safety_margin:
            return False
    return True


def segment_crosses_footprint(x1, y1, x2, y2, obj, safety_margin):
    distance = math.hypot(x2 - x1, y2 - y1)
    samples = max(8, min(80, int(distance / 0.15)))
    for index in range(samples + 1):
        ratio = index / samples
        x = x1 + (x2 - x1) * ratio
        y = y1 + (y2 - y1) * ratio
        if point_to_footprint_distance(x, y, obj) <= safety_margin:
            return True
    return False


def has_line_of_sight(camera_pose, target, los_blockers, safety_margin):
    for blocker in los_blockers:
        if blocker.name == target.name:
            continue
        if point_to_footprint_distance(target.pose.x, target.pose.y, blocker) <= safety_margin:
            continue
        if segment_crosses_footprint(
            camera_pose.x,
            camera_pose.y,
            target.pose.x,
            target.pose.y,
            blocker,
            safety_margin,
        ):
            return False
    return True


class PosePlanner:
    def __init__(
        self,
        objects,
        targets,
        background_anchors,
        obstacles,
        los_blockers,
        camera_height,
        safety_margin,
        max_attempts,
        seed,
    ):
        self.objects = objects
        self.targets = targets
        self.background_anchors = background_anchors
        self.obstacles = obstacles
        self.los_blockers = los_blockers
        self.camera_height = camera_height
        self.safety_margin = safety_margin
        self.max_attempts = max_attempts
        self.rng = random.Random(seed)
        self.bounds = self._compute_bounds(objects)

    def _compute_bounds(self, objects):
        xs = [
            obj.pose.x
            for obj in objects
            if abs(obj.pose.x) < 100.0
            and abs(obj.pose.y) < 100.0
            and not any(key in obj.name.lower() for key in ("ground", "road_h", "road_v"))
        ]
        ys = [
            obj.pose.y
            for obj in objects
            if abs(obj.pose.x) < 100.0
            and abs(obj.pose.y) < 100.0
            and not any(key in obj.name.lower() for key in ("ground", "road_h", "road_v"))
        ]
        if not xs or not ys:
            return (-35.0, 35.0, -35.0, 35.0)
        return (
            clamp(min(xs) - 4.0, -42.0, 42.0),
            clamp(max(xs) + 4.0, -42.0, 42.0),
            clamp(min(ys) - 4.0, -42.0, 42.0),
            clamp(max(ys) + 4.0, -42.0, 42.0),
        )

    def plan(self, mode):
        class_name = TARGET_MODES.get(mode)
        for attempt in range(1, self.max_attempts + 1):
            if class_name and self.targets.get(class_name):
                target = self.rng.choice(self.targets[class_name])
                pose, distance = self._target_pose(class_name, target)
                if distance < 0.7:
                    continue
                if not point_is_collision_free(pose.x, pose.y, self.obstacles, self.safety_margin):
                    continue
                if not has_line_of_sight(pose, target, self.los_blockers, self.safety_margin):
                    continue
                return PlannedShot(pose, mode, target, attempt, "ok")

            else:
                pose = self._background_pose()
                if not point_is_collision_free(pose.x, pose.y, self.obstacles, self.safety_margin):
                    continue
                return PlannedShot(pose, "background_random", None, attempt, "ok")

        return None

    def _target_pose(self, class_name, target):
        distance_ranges = ((1.0, 2.0), (2.0, 4.0), (4.0, 8.0))
        low, high = self.rng.choice(distance_ranges)
        distance = self.rng.uniform(low, high)
        angle = self.rng.uniform(-math.pi, math.pi)
        x = target.pose.x + distance * math.cos(angle)
        y = target.pose.y + distance * math.sin(angle)
        z = clamp(self.camera_height + self.rng.uniform(-0.02, 0.0), 0.30, 0.35)
        yaw = math.atan2(target.pose.y - y, target.pose.x - x)
        yaw = normalize_angle(yaw + self.rng.uniform(-0.35, 0.35))
        pitch = self.rng.uniform(-0.08, 0.08)
        return Pose6D(x=x, y=y, z=z, roll=0.0, pitch=pitch, yaw=yaw), distance

    def _background_pose(self):
        min_x, max_x, min_y, max_y = self.bounds
        if self.background_anchors and self.rng.random() < 0.75:
            anchor = self.rng.choice(self.background_anchors)
            x = clamp(anchor.pose.x + self.rng.uniform(-7.0, 7.0), min_x, max_x)
            y = clamp(anchor.pose.y + self.rng.uniform(-7.0, 7.0), min_y, max_y)
        else:
            x = self.rng.uniform(min_x, max_x)
            y = self.rng.uniform(min_y, max_y)

        return Pose6D(
            x=x,
            y=y,
            z=clamp(self.camera_height + self.rng.uniform(-0.02, 0.0), 0.30, 0.35),
            roll=0.0,
            pitch=self.rng.uniform(-0.08, 0.08),
            yaw=self.rng.uniform(-math.pi, math.pi),
        )


def make_camera_intrinsics(width, height, horizontal_fov):
    fx = width / (2.0 * math.tan(horizontal_fov / 2.0))
    vertical_fov = 2.0 * math.atan(math.tan(horizontal_fov / 2.0) * height / width)
    fy = height / (2.0 * math.tan(vertical_fov / 2.0))
    return CameraIntrinsics(
        width=width,
        height=height,
        horizontal_fov=horizontal_fov,
        vertical_fov=vertical_fov,
        fx=fx,
        fy=fy,
        cx=width * 0.5,
        cy=height * 0.5,
    )


def proxy_dimensions(obj):
    spec = OBJECT_PROXY_SIZES[obj.class_name]
    if obj.class_name == "person":
        width = clamp(max(obj.size_x, obj.size_y, spec["width"]), 0.35, 0.90)
        depth = clamp(min(max(min(obj.size_x, obj.size_y), spec["depth"]), 0.60), 0.20, 0.60)
        height = clamp(obj.size_z if obj.size_z > 0 else spec["height"], 1.45, 2.05)
    elif obj.class_name == "traffic_light":
        width = clamp(max(obj.size_x, obj.size_y, spec["width"]), 0.25, 0.80)
        depth = clamp(min(max(min(obj.size_x, obj.size_y), spec["depth"]), 0.50), 0.12, 0.50)
        height = clamp(obj.size_z if obj.size_z > 0.4 else spec["height"], 0.8, 1.8)
    else:
        width = spec["width"]
        depth = spec["depth"]
        height = spec["height"]
    return width, depth, height, spec["z_offset"]


def proxy_points_world(obj):
    width, depth, height, z_offset = proxy_dimensions(obj)
    center_z = obj.pose.z if obj.class_name == "apriltag" else obj.pose.z + z_offset
    center = (obj.pose.x, obj.pose.y, center_z)
    rotation = rotation_matrix_from_euler(obj.pose.roll, obj.pose.pitch, obj.pose.yaw)
    points = []
    for local_x in (-width * 0.5, width * 0.5):
        for local_y in (-depth * 0.5, depth * 0.5):
            for local_z in (-height * 0.5, height * 0.5):
                rotated = mat_vec_mul(rotation, (local_x, local_y, local_z))
                points.append(
                    (
                        center[0] + rotated[0],
                        center[1] + rotated[1],
                        center[2] + rotated[2],
                    )
                )
    return points


def world_to_camera_optical(points_world, camera_pose):
    rotation = rotation_matrix_from_euler(camera_pose.roll, camera_pose.pitch, camera_pose.yaw)
    points_optical = []
    for point in points_world:
        relative_world = (
            point[0] - camera_pose.x,
            point[1] - camera_pose.y,
            point[2] - camera_pose.z,
        )
        body_x, body_y, body_z = mat_transpose_vec_mul(rotation, relative_world)
        # Gazebo camera poses in this project use body +X as forward, +Y as left,
        # and +Z as up. The pinhole projection below uses optical axes:
        # X_optical = right, Y_optical = down, Z_optical = forward.
        points_optical.append((-body_y, -body_z, body_x))
    return points_optical


def project_world_points_to_image(points_world, camera_pose, intrinsics):
    projected = []
    for x_opt, y_opt, z_opt in world_to_camera_optical(points_world, camera_pose):
        if z_opt <= 1e-6:
            projected.append((float("nan"), float("nan"), z_opt))
            continue
        u = intrinsics.fx * x_opt / z_opt + intrinsics.cx
        v = intrinsics.fy * y_opt / z_opt + intrinsics.cy
        projected.append((u, v, z_opt))
    return projected


class ApproxLabeler:
    def __init__(
        self,
        target_objects,
        los_blockers,
        width,
        height,
        horizontal_fov,
        min_projected_bbox_size,
        max_labels_per_image,
        max_label_distance,
        safety_margin,
    ):
        self.target_objects = target_objects
        self.los_blockers = los_blockers
        self.width = width
        self.height = height
        self.horizontal_fov = horizontal_fov
        self.min_projected_bbox_size = min_projected_bbox_size
        self.max_labels_per_image = max_labels_per_image
        self.max_label_distance = max_label_distance
        self.safety_margin = safety_margin
        self.intrinsics = make_camera_intrinsics(width, height, horizontal_fov)

    def labels_for_pose(self, camera_pose, target_object=None):
        candidates = []
        rejections = []
        num_projected_objects = 0

        for obj in self.target_objects:
            label, rejection, projected = self._label_object(camera_pose, obj, target_object)
            if projected:
                num_projected_objects += 1
            if label is not None:
                candidates.append(label)
            elif rejection is not None:
                rejections.append(rejection)

        candidates.sort(key=lambda label: label.score, reverse=True)
        labels = candidates[: self.max_labels_per_image]
        labels.sort(key=lambda label: (label.class_id, label.x_center))
        target_was_labeled = (
            target_object is not None
            and any(label.object_name == target_object.name for label in labels)
        )
        return LabelResult(
            labels=labels,
            rejections=rejections,
            num_candidate_objects=len(self.target_objects),
            num_projected_objects=num_projected_objects,
            target_was_labeled=target_was_labeled,
        )

    def _reject(self, obj, distance, reason):
        return ProjectionRejection(
            object_name=obj.name,
            class_name=obj.class_name or "",
            distance=distance,
            reason=reason,
        )

    def _label_object(self, camera_pose, obj, target_object):
        dx = obj.pose.x - camera_pose.x
        dy = obj.pose.y - camera_pose.y
        distance = math.hypot(dx, dy)
        if distance < 0.4:
            return None, self._reject(obj, distance, "too_close"), False
        if distance > self.max_label_distance:
            return None, self._reject(obj, distance, "too_far"), False
        if not has_line_of_sight(camera_pose, obj, self.los_blockers, self.safety_margin):
            return None, self._reject(obj, distance, "occluded"), False

        points_world = proxy_points_world(obj)
        projected = project_world_points_to_image(points_world, camera_pose, self.intrinsics)
        visible_points = [(u, v, z_cam) for u, v, z_cam in projected if z_cam > 0.05]
        if not visible_points:
            return None, self._reject(obj, distance, "behind_camera"), False
        if len(visible_points) < 2:
            return None, self._reject(obj, distance, "invalid_projection"), True

        min_u = min(point[0] for point in visible_points)
        max_u = max(point[0] for point in visible_points)
        min_v = min(point[1] for point in visible_points)
        max_v = max(point[1] for point in visible_points)
        raw_width = max_u - min_u
        raw_height = max_v - min_v
        raw_area = raw_width * raw_height
        if raw_area <= 0.0 or not all(math.isfinite(value) for value in (min_u, max_u, min_v, max_v)):
            return None, self._reject(obj, distance, "invalid_projection"), True

        raw_center_u = (min_u + max_u) * 0.5
        raw_center_v = (min_v + max_v) * 0.5
        if (
            raw_center_u < -self.width * 0.5
            or raw_center_u > self.width * 1.5
            or raw_center_v < -self.height * 0.5
            or raw_center_v > self.height * 1.5
        ):
            return None, self._reject(obj, distance, "outside_fov"), True

        x1 = clamp(min_u, 0.0, self.width - 1.0)
        x2 = clamp(max_u, 0.0, self.width - 1.0)
        y1 = clamp(min_v, 0.0, self.height - 1.0)
        y2 = clamp(max_v, 0.0, self.height - 1.0)
        clipped_width = x2 - x1
        clipped_height = y2 - y1
        clipped_area = clipped_width * clipped_height

        if clipped_width < self.min_projected_bbox_size or clipped_height < self.min_projected_bbox_size:
            return None, self._reject(obj, distance, "too_small"), True
        if clipped_area < self.min_projected_bbox_size * self.min_projected_bbox_size:
            return None, self._reject(obj, distance, "too_small"), True
        if clipped_width > self.width * 0.85 or clipped_height > self.height * 0.95:
            return None, self._reject(obj, distance, "too_large"), True
        if clipped_area / raw_area < 0.25:
            return None, self._reject(obj, distance, "clipped_too_much"), True

        image_center_u = self.width * 0.5
        image_center_v = self.height * 0.5
        bbox_center_u = (x1 + x2) * 0.5
        bbox_center_v = (y1 + y2) * 0.5
        center_distance = math.hypot(
            (bbox_center_u - image_center_u) / self.width,
            (bbox_center_v - image_center_v) / self.height,
        )
        center_bonus = max(0.2, 1.2 - center_distance)
        area_score = clipped_area / (distance + 0.5)
        score = area_score * center_bonus
        if target_object is not None and obj.name == target_object.name:
            score += 100000.0
        if clipped_area < 400.0:
            score *= 0.6
        if clipped_width > self.width * 0.65 or clipped_height > self.height * 0.75:
            score *= 0.65

        return YoloLabel(
            class_id=CLASS_SPECS[obj.class_name]["id"],
            class_name=obj.class_name,
            object_name=obj.name,
            x_center=((x1 + x2) * 0.5) / self.width,
            y_center=((y1 + y2) * 0.5) / self.height,
            width=clipped_width / self.width,
            height=clipped_height / self.height,
            distance=distance,
            width_px=clipped_width,
            height_px=clipped_height,
            score=score,
        ), None, True


class GazeboPoseClient:
    def __init__(self, world_name, model_name, timeout_sec):
        self.world_name = world_name
        self.model_name = model_name
        self.timeout_sec = timeout_sec
        self.gz_executable = shutil.which("gz") or shutil.which("ign")
        if self.gz_executable is None:
            raise RuntimeError("Neither 'gz' nor 'ign' executable was found in PATH.")

    def set_pose(self, pose):
        qx, qy, qz, qw = quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)
        request = (
            f'name: "{self.model_name}" '
            f"position {{ x: {pose.x:.8f} y: {pose.y:.8f} z: {pose.z:.8f} }} "
            f"orientation {{ x: {qx:.10f} y: {qy:.10f} z: {qz:.10f} w: {qw:.10f} }}"
        )
        command = [
            self.gz_executable,
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(int(self.timeout_sec * 1000)),
            "--req",
            request,
        ]
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        output = f"{result.stdout}\n{result.stderr}".strip()
        if result.returncode != 0:
            raise RuntimeError(
                f"Gazebo set_pose failed with exit code {result.returncode}: {output}"
            )
        if "data: false" in output.lower():
            raise RuntimeError(f"Gazebo set_pose returned false: {output}")


class DatasetCollector(Node):
    def __init__(self, args):
        super().__init__("city_cv_dataset_collector")
        self.args = args
        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.last_image = None
        self.last_image_seq = 0
        self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def image_callback(self, msg):
        with self.image_lock:
            self.last_image = msg
            self.last_image_seq += 1

    def wait_for_fresh_image(self, previous_seq, min_new_frames=1):
        deadline = time.monotonic() + self.args.image_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self.image_lock:
                if self.last_image is not None and self.last_image_seq >= previous_seq + min_new_frames:
                    return self.last_image, self.last_image_seq
        raise TimeoutError(
            f"No fresh image received from {self.args.image_topic} "
            f"within {self.args.image_timeout:.1f}s."
        )

    def current_image_seq(self):
        with self.image_lock:
            return self.last_image_seq

    def image_msg_to_bgr(self, msg):
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            encoding = (msg.encoding or "").lower()
            if len(image.shape) == 3 and image.shape[2] == 3 and encoding in ("rgb8", "r8g8b8"):
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 4 and encoding == "rgba8":
                image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            return image

    def write_dataset_yaml(self, output_dir):
        dataset_yaml = output_dir / "dataset.yaml"
        path_value = str(output_dir)
        repo_relative = Path("datasets") / "city_cv_yolo_raw"
        if output_dir.name == "city_cv_yolo_raw" and output_dir.parent.name == "datasets":
            path_value = str(repo_relative)

        dataset_yaml.write_text(
            "\n".join(
                [
                    f"path: {path_value}",
                    "train: images",
                    "val: images",
                    "",
                    "names:",
                    "  0: person",
                    "  1: traffic_light",
                    "  2: apriltag",
                    "",
                ]
            ),
            encoding="utf-8",
        )

    def save_labels(self, labels, path):
        with path.open("w", encoding="utf-8") as label_file:
            for label in labels:
                label_file.write(
                    f"{label.class_id} "
                    f"{label.x_center:.6f} {label.y_center:.6f} "
                    f"{label.width:.6f} {label.height:.6f}\n"
                )

    def save_debug_preview(self, image, labels, path):
        preview = image.copy()
        for label in labels:
            x_center = label.x_center * self.args.width
            y_center = label.y_center * self.args.height_image
            box_width = label.width * self.args.width
            box_height = label.height * self.args.height_image
            x1 = int(round(x_center - box_width * 0.5))
            y1 = int(round(y_center - box_height * 0.5))
            x2 = int(round(x_center + box_width * 0.5))
            y2 = int(round(y_center + box_height * 0.5))
            color = CLASS_SPECS[label.class_name]["color"]
            cv2.rectangle(preview, (x1, y1), (x2, y2), color, 3)
            cv2.circle(preview, (int(round(x_center)), int(round(y_center))), 4, color, -1)
            caption = (
                f"{label.class_name} d={label.distance:.1f} "
                f"w={label.width_px:.0f} h={label.height_px:.0f}"
            )
            text_y = max(14, y1 - 5)
            cv2.putText(
                preview,
                caption,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA,
            )
        if not cv2.imwrite(str(path), preview):
            raise RuntimeError(f"Failed to write debug preview to {path}")

    def save_rejection_debug(self, rejections, path):
        with path.open("w", encoding="utf-8") as debug_file:
            debug_file.write("object_name,class,distance,reason\n")
            for rejection in rejections:
                debug_file.write(
                    f"{rejection.object_name},{rejection.class_name},"
                    f"{rejection.distance:.3f},{rejection.reason}\n"
                )

    def collect(self):
        world_path = Path(self.args.world_path).expanduser().resolve()
        output_dir = Path(self.args.output_dir).expanduser().resolve()
        images_dir = output_dir / "images"
        labels_dir = output_dir / "labels"
        debug_dir = output_dir / "debug_preview"
        metadata_path = output_dir / "metadata.csv"

        images_dir.mkdir(parents=True, exist_ok=True)
        labels_dir.mkdir(parents=True, exist_ok=True)
        debug_dir.mkdir(parents=True, exist_ok=True)
        self.write_dataset_yaml(output_dir)

        if self.args.world_name == "auto":
            self.args.world_name = read_world_name(world_path)

        objects = load_world_objects(world_path)
        targets, background_anchors, obstacles, los_blockers = categorize_objects(objects)
        target_objects = [
            target
            for class_targets in targets.values()
            for target in class_targets
        ]
        counts = allocate_counts(self.args.num_images, targets, self.get_logger())
        modes = [
            mode
            for mode, count in counts.items()
            for _ in range(count)
        ]
        rng = random.Random(self.args.seed)
        rng.shuffle(modes)

        self.get_logger().info(f"Loaded {len(objects)} world objects from {world_path}")
        for class_name in CLASS_SPECS:
            self.get_logger().info(f"Found {len(targets[class_name])} {class_name} target objects")
        self.get_logger().info(f"Found {len(obstacles)} collision obstacles")
        self.get_logger().info(f"Found {len(los_blockers)} line-of-sight blockers")
        self.get_logger().info(f"Frame allocation: {counts}")
        self.get_logger().info(f"Writing dataset to {output_dir}")

        planner = PosePlanner(
            objects=objects,
            targets=targets,
            background_anchors=background_anchors,
            obstacles=obstacles,
            los_blockers=los_blockers,
            camera_height=self.args.camera_height,
            safety_margin=self.args.safety_margin,
            max_attempts=self.args.max_attempts_per_image,
            seed=self.args.seed,
        )
        labeler = ApproxLabeler(
            target_objects=target_objects,
            los_blockers=los_blockers,
            width=self.args.width,
            height=self.args.height_image,
            horizontal_fov=self.args.horizontal_fov,
            min_projected_bbox_size=self.args.min_projected_bbox_size,
            max_labels_per_image=self.args.max_labels_per_image,
            max_label_distance=self.args.max_label_distance,
            safety_margin=self.args.safety_margin,
        )
        pose_client = GazeboPoseClient(
            world_name=self.args.world_name,
            model_name=self.args.camera_model_name,
            timeout_sec=self.args.pose_timeout,
        )

        self.get_logger().info(f"Waiting for initial image on {self.args.image_topic}")
        self.wait_for_fresh_image(previous_seq=0, min_new_frames=1)

        with metadata_path.open("w", newline="", encoding="utf-8") as metadata_file:
            writer = csv.DictWriter(
                metadata_file,
                fieldnames=[
                    "filename",
                    "image_path",
                    "label_path",
                    "camera_x",
                    "camera_y",
                    "camera_z",
                    "camera_roll",
                    "camera_pitch",
                    "camera_yaw",
                    "generation_mode",
                    "target_object_name",
                    "target_class",
                    "target_distance",
                    "num_labels",
                    "num_candidate_objects",
                    "num_projected_objects",
                    "num_saved_labels",
                    "label_quality_mode",
                    "target_was_labeled",
                    "timestamp",
                    "valid_pose_attempts",
                    "notes",
                ],
            )
            writer.writeheader()

            saved_count = 0
            skipped_count = 0
            mode_index = 0
            while saved_count < self.args.num_images and mode_index < len(modes):
                mode = modes[mode_index]
                mode_index += 1
                planned = planner.plan(mode)
                if planned is None:
                    skipped_count += 1
                    self.get_logger().warning(
                        f"Skipping requested frame {mode_index}: no valid pose for mode={mode}"
                    )
                    continue

                saved_count += 1
                filename = f"img_{saved_count:06d}.png"
                image_path = images_dir / filename
                label_path = labels_dir / filename.replace(".png", ".txt")
                debug_path = debug_dir / filename.replace(".png", "_debug.png")
                rejection_path = debug_dir / filename.replace(".png", "_rejections.txt")

                last_error = None
                for attempt in range(1, self.args.max_capture_retries + 1):
                    try:
                        previous_seq = self.current_image_seq()
                        pose_client.set_pose(planned.pose)
                        time.sleep(self.args.settle_time)
                        msg, _ = self.wait_for_fresh_image(
                            previous_seq=previous_seq,
                            min_new_frames=self.args.fresh_frames,
                        )
                        image = self.image_msg_to_bgr(msg)
                        if not cv2.imwrite(str(image_path), image):
                            raise RuntimeError(f"Failed to write image to {image_path}")
                        break
                    except Exception as exc:
                        last_error = exc
                        self.get_logger().warning(
                            f"Retry {attempt}/{self.args.max_capture_retries} for {filename}: {exc}"
                        )
                        time.sleep(0.2)
                else:
                    raise RuntimeError(f"Failed to collect {filename}: {last_error}")

                label_result = labeler.labels_for_pose(planned.pose, planned.target)
                self.save_labels(label_result.labels, label_path)
                if saved_count <= self.args.debug_preview_count:
                    self.save_debug_preview(image, label_result.labels, debug_path)
                    if self.args.save_rejection_debug:
                        self.save_rejection_debug(label_result.rejections, rejection_path)

                timestamp = datetime.now(timezone.utc).isoformat()
                target_distance = ""
                target_name = ""
                target_class = ""
                if planned.target is not None:
                    target_name = planned.target.name
                    target_class = planned.target.class_name or ""
                    target_distance = f"{math.hypot(planned.target.pose.x - planned.pose.x, planned.target.pose.y - planned.pose.y):.6f}"

                writer.writerow(
                    {
                        "filename": filename,
                        "image_path": str(image_path),
                        "label_path": str(label_path),
                        "camera_x": f"{planned.pose.x:.6f}",
                        "camera_y": f"{planned.pose.y:.6f}",
                        "camera_z": f"{planned.pose.z:.6f}",
                        "camera_roll": f"{planned.pose.roll:.6f}",
                        "camera_pitch": f"{planned.pose.pitch:.6f}",
                        "camera_yaw": f"{planned.pose.yaw:.6f}",
                        "generation_mode": planned.mode,
                        "target_object_name": target_name,
                        "target_class": target_class,
                        "target_distance": target_distance,
                        "num_labels": len(label_result.labels),
                        "num_candidate_objects": label_result.num_candidate_objects,
                        "num_projected_objects": label_result.num_projected_objects,
                        "num_saved_labels": len(label_result.labels),
                        "label_quality_mode": "projected_proxy_bbox",
                        "target_was_labeled": str(label_result.target_was_labeled).lower(),
                        "timestamp": timestamp,
                        "valid_pose_attempts": planned.attempts,
                        "notes": planned.notes,
                    }
                )
                metadata_file.flush()
                self.get_logger().info(
                    f"[{saved_count}/{self.args.num_images}] mode={planned.mode} "
                    f"labels={len(label_result.labels)} saved={image_path}"
                )

            if saved_count < self.args.num_images:
                self.get_logger().warning(
                    f"Saved {saved_count}/{self.args.num_images} images; "
                    f"skipped {skipped_count} requested frames after pose planning failures."
                )


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def positive_float(value):
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Collect approximate YOLO labels and RGB images from the city_cv Gazebo world."
    )
    parser.add_argument("--world-path", default=default_world_path())
    parser.add_argument("--world-name", default="city_second")
    parser.add_argument("--output-dir", default=default_output_dir())
    parser.add_argument("--num-images", type=positive_int, default=6000)
    parser.add_argument("--width", type=positive_int, default=640)
    parser.add_argument("--height-image", type=positive_int, default=360)
    parser.add_argument("--camera-height", type=positive_float, default=0.35)
    parser.add_argument("--horizontal-fov", type=positive_float, default=1.6)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--debug-preview-count", type=int, default=200)
    parser.add_argument("--safety-margin", type=positive_float, default=0.35)
    parser.add_argument("--max-attempts-per-image", type=positive_int, default=200)
    parser.add_argument("--min-bbox-size-px", type=positive_int, default=4)
    parser.add_argument("--max-labels-per-image", type=positive_int, default=3)
    parser.add_argument("--max-label-distance", type=positive_float, default=12.0)
    parser.add_argument("--min-projected-bbox-size", type=positive_int, default=6)
    parser.add_argument("--save-rejection-debug", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--image-topic", default="/dataset_camera/color/image_raw")
    parser.add_argument("--camera-model-name", default="dataset_camera")
    parser.add_argument("--pose-timeout", type=positive_float, default=3.0)
    parser.add_argument("--image-timeout", type=positive_float, default=10.0)
    parser.add_argument("--settle-time", type=float, default=0.2)
    parser.add_argument("--fresh-frames", type=positive_int, default=1)
    parser.add_argument("--max-capture-retries", type=positive_int, default=3)
    return parser


def main(argv=None):
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args(argv)
    args.debug_preview_count = max(0, args.debug_preview_count)

    rclpy.init(args=ros_args)
    node = DatasetCollector(args)
    try:
        node.collect()
    except KeyboardInterrupt:
        node.get_logger().warning("Dataset collection interrupted by user.")
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
