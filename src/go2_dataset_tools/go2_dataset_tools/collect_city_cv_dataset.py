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


DESIRED_DISTRIBUTION = {
    "people": 2000,
    "traffic_light": 1500,
    "apriltag": 1500,
    "background": 1000,
}

PEOPLE_KEYS = ("person", "male", "female")
TRAFFIC_LIGHT_KEYS = ("traffic_light", "traffic", "light")
APRILTAG_KEYS = ("apriltag", "april_tag", "april-tag", "marker", "target_marker", "tag")
BACKGROUND_ANCHOR_KEYS = ("road", "crosswalk", "sidewalk", "spawn")
TARGET_CATEGORIES = ("people", "traffic_light", "apriltag")


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


def _strip_namespace(tag):
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def _direct_child(element, child_name):
    for child in element:
        if _strip_namespace(child.tag) == child_name:
            return child
    return None


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


def load_world_objects(world_path):
    try:
        root = ET.parse(world_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f"Failed to parse SDF world '{world_path}': {exc}") from exc

    world = root if _strip_namespace(root.tag) == "world" else None
    if world is None:
        for child in root:
            if _strip_namespace(child.tag) == "world":
                world = child
                break
    if world is None:
        raise RuntimeError(f"No <world> element found in SDF '{world_path}'.")

    objects = []
    for child in world:
        tag = _strip_namespace(child.tag)
        if tag == "model":
            name = child.attrib.get("name", "").strip()
            if name:
                objects.append(WorldObject(name=name, pose=read_pose(child)))
        elif tag == "include":
            name = read_text_child(child, "name")
            if not name:
                uri = read_text_child(child, "uri")
                name = Path(uri.rstrip("/")).name if uri else "include"
            objects.append(WorldObject(name=name, pose=read_pose(child)))
    return objects


def categorize_objects(objects):
    targets = {category: [] for category in TARGET_CATEGORIES}
    background_anchors = []

    for obj in objects:
        name = obj.name.lower()
        if any(key in name for key in PEOPLE_KEYS):
            targets["people"].append(obj)
        elif any(key in name for key in TRAFFIC_LIGHT_KEYS):
            targets["traffic_light"].append(obj)
        elif any(key in name for key in APRILTAG_KEYS):
            targets["apriltag"].append(obj)

        if any(key in name for key in BACKGROUND_ANCHOR_KEYS):
            background_anchors.append(obj)

    return targets, background_anchors


def allocate_counts(num_images, targets, logger):
    available_weights = {}
    for category, desired in DESIRED_DISTRIBUTION.items():
        if category == "background" or targets.get(category):
            available_weights[category] = desired
        else:
            logger.warning(
                f"No SDF objects found for '{category}', redistributing its frames."
            )

    if not available_weights:
        available_weights["background"] = num_images

    weight_sum = float(sum(available_weights.values()))
    raw_counts = {
        category: (num_images * weight / weight_sum)
        for category, weight in available_weights.items()
    }
    counts = {category: int(math.floor(value)) for category, value in raw_counts.items()}
    remaining = num_images - sum(counts.values())
    by_fraction = sorted(
        raw_counts,
        key=lambda category: raw_counts[category] - counts[category],
        reverse=True,
    )
    for category in by_fraction[:remaining]:
        counts[category] += 1

    for category in DESIRED_DISTRIBUTION:
        counts.setdefault(category, 0)
    return counts


def clamp(value, low, high):
    return max(low, min(high, value))


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


class PosePlanner:
    def __init__(self, objects, targets, background_anchors, height, seed):
        self.objects = objects
        self.targets = targets
        self.background_anchors = background_anchors
        self.height = height
        self.rng = random.Random(seed)
        self.bounds = self._compute_bounds(objects)
        self.all_targets = [
            target
            for category in TARGET_CATEGORIES
            for target in self.targets.get(category, [])
        ]

    def _compute_bounds(self, objects):
        xs = [obj.pose.x for obj in objects if abs(obj.pose.x) < 100.0 and abs(obj.pose.y) < 100.0]
        ys = [obj.pose.y for obj in objects if abs(obj.pose.x) < 100.0 and abs(obj.pose.y) < 100.0]
        if not xs or not ys:
            return (-35.0, 35.0, -35.0, 35.0)
        return (
            clamp(min(xs) - 2.0, -42.0, 42.0),
            clamp(max(xs) + 2.0, -42.0, 42.0),
            clamp(min(ys) - 2.0, -42.0, 42.0),
            clamp(max(ys) + 2.0, -42.0, 42.0),
        )

    def camera_height(self):
        return clamp(self.height + self.rng.uniform(-0.025, 0.015), 0.30, 0.35)

    def next_pose(self, category):
        if category in TARGET_CATEGORIES and self.targets.get(category):
            return self._target_pose(category)
        return self._background_pose()

    def _target_pose(self, category):
        target = self.rng.choice(self.targets[category])
        distances = [1.0, 2.0, 3.5, 5.0, 7.0]
        distance = self.rng.choice(distances)
        if self.rng.random() < 0.15:
            distance = self.rng.uniform(7.0, 10.0)

        if category == "traffic_light":
            angle = target.pose.yaw + self.rng.choice(
                [0.0, math.pi, math.pi / 2.0, -math.pi / 2.0, 0.6, -0.6]
            )
            angle += self.rng.uniform(-0.35, 0.35)
        else:
            angle = self.rng.uniform(-math.pi, math.pi)

        x = target.pose.x + distance * math.cos(angle) + self.rng.gauss(0.0, 0.08)
        y = target.pose.y + distance * math.sin(angle) + self.rng.gauss(0.0, 0.08)
        z = self.camera_height()

        yaw = math.atan2(target.pose.y - y, target.pose.x - x)
        yaw += self.rng.gauss(0.0, 0.08)
        if self.rng.random() < 0.25:
            yaw += self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.18, 0.55)

        horizontal_distance = max(0.1, math.hypot(target.pose.x - x, target.pose.y - y))
        aim_z = self._target_aim_z(category, target)
        pitch = -math.atan2(aim_z - z, horizontal_distance)
        pitch += self.rng.gauss(0.0, 0.035)
        pitch = clamp(pitch, -0.45, 0.22)
        roll = clamp(self.rng.gauss(0.0, 0.015), -0.04, 0.04)
        return Pose6D(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)

    def _target_aim_z(self, category, target):
        if category == "people":
            return max(target.pose.z + 1.0, 0.9)
        if category == "traffic_light":
            return max(target.pose.z + 1.45, 1.2)
        return max(target.pose.z, 0.25)

    def _background_pose(self):
        min_x, max_x, min_y, max_y = self.bounds
        for _ in range(100):
            if self.background_anchors and self.rng.random() < 0.85:
                anchor = self.rng.choice(self.background_anchors)
                x = anchor.pose.x + self.rng.uniform(-4.5, 4.5)
                y = anchor.pose.y + self.rng.uniform(-4.5, 4.5)
                x = clamp(x, min_x, max_x)
                y = clamp(y, min_y, max_y)
            else:
                x = self.rng.uniform(min_x, max_x)
                y = self.rng.uniform(min_y, max_y)

            if self._far_from_targets(x, y, min_distance=6.0):
                break
        else:
            x = self.rng.uniform(min_x, max_x)
            y = self.rng.uniform(min_y, max_y)

        return Pose6D(
            x=x,
            y=y,
            z=self.camera_height(),
            roll=clamp(self.rng.gauss(0.0, 0.015), -0.04, 0.04),
            pitch=self.rng.uniform(-0.12, 0.08),
            yaw=self.rng.uniform(-math.pi, math.pi),
        )

    def _far_from_targets(self, x, y, min_distance):
        for target in self.all_targets:
            if math.hypot(target.pose.x - x, target.pose.y - y) < min_distance:
                return False
        return True


class GazeboPoseClient:
    def __init__(self, world_name, model_name, timeout_sec, logger):
        self.world_name = world_name
        self.model_name = model_name
        self.timeout_sec = timeout_sec
        self.logger = logger
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

    def save_image(self, msg, path):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            encoding = (msg.encoding or "").lower()
            if len(image.shape) == 3 and image.shape[2] == 3 and encoding in ("rgb8", "rgba8"):
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f"Failed to write image to {path}")

    def collect(self):
        world_path = Path(self.args.world_path).expanduser().resolve()
        output_dir = Path(self.args.output_dir).expanduser().resolve()
        images_dir = output_dir / "images"
        metadata_path = output_dir / "metadata.csv"
        images_dir.mkdir(parents=True, exist_ok=True)

        objects = load_world_objects(world_path)
        targets, background_anchors = categorize_objects(objects)
        counts = allocate_counts(self.args.num_images, targets, self.get_logger())
        modes = [
            category
            for category, count in counts.items()
            for _ in range(count)
        ]
        rng = random.Random(self.args.seed)
        rng.shuffle(modes)

        self.get_logger().info(f"Loaded {len(objects)} world objects from {world_path}")
        for category in TARGET_CATEGORIES:
            self.get_logger().info(f"Found {len(targets[category])} {category} target objects")
        self.get_logger().info(f"Frame allocation: {counts}")
        self.get_logger().info(f"Writing images to {images_dir}")
        self.get_logger().info(f"Writing metadata to {metadata_path}")

        planner = PosePlanner(
            objects=objects,
            targets=targets,
            background_anchors=background_anchors,
            height=self.args.height,
            seed=self.args.seed,
        )
        pose_client = GazeboPoseClient(
            world_name=self.args.world_name,
            model_name=self.args.camera_model_name,
            timeout_sec=self.args.pose_timeout,
            logger=self.get_logger(),
        )

        self.get_logger().info(f"Waiting for initial image on {self.args.image_topic}")
        self.wait_for_fresh_image(previous_seq=0, min_new_frames=1)

        with metadata_path.open("w", newline="", encoding="utf-8") as metadata_file:
            writer = csv.DictWriter(
                metadata_file,
                fieldnames=[
                    "filename",
                    "x",
                    "y",
                    "z",
                    "roll",
                    "pitch",
                    "yaw",
                    "mode",
                    "timestamp",
                ],
            )
            writer.writeheader()

            for index, mode in enumerate(modes, start=1):
                filename = f"img_{index:06d}.png"
                image_path = images_dir / filename
                pose = planner.next_pose(mode)

                last_error = None
                for attempt in range(1, self.args.max_retries + 1):
                    try:
                        previous_seq = self.current_image_seq()
                        pose_client.set_pose(pose)
                        time.sleep(self.args.settle_time)
                        msg, _ = self.wait_for_fresh_image(
                            previous_seq=previous_seq,
                            min_new_frames=self.args.fresh_frames,
                        )
                        self.save_image(msg, image_path)
                        break
                    except Exception as exc:
                        last_error = exc
                        self.get_logger().warning(
                            f"Retry {attempt}/{self.args.max_retries} for {filename}: {exc}"
                        )
                        time.sleep(0.2)
                else:
                    raise RuntimeError(f"Failed to collect {filename}: {last_error}")

                timestamp = datetime.now(timezone.utc).isoformat()
                writer.writerow(
                    {
                        "filename": filename,
                        "x": f"{pose.x:.6f}",
                        "y": f"{pose.y:.6f}",
                        "z": f"{pose.z:.6f}",
                        "roll": f"{pose.roll:.6f}",
                        "pitch": f"{pose.pitch:.6f}",
                        "yaw": f"{pose.yaw:.6f}",
                        "mode": mode,
                        "timestamp": timestamp,
                    }
                )
                metadata_file.flush()
                self.get_logger().info(
                    f"[{index}/{self.args.num_images}] mode={mode} saved={image_path}"
                )


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Collect city_cv Gazebo RGB images for YOLO dataset pre-labeling."
    )
    parser.add_argument("--world-path", default=default_world_path())
    parser.add_argument("--world-name", default="city_second")
    parser.add_argument("--output-dir", default=default_output_dir())
    parser.add_argument("--num-images", type=positive_int, default=6000)
    parser.add_argument("--height", type=float, default=0.35)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--image-topic", default="/dataset_camera/color/image_raw")
    parser.add_argument("--camera-model-name", default="dataset_camera")
    parser.add_argument("--pose-timeout", type=float, default=3.0)
    parser.add_argument("--image-timeout", type=float, default=10.0)
    parser.add_argument("--settle-time", type=float, default=0.12)
    parser.add_argument("--fresh-frames", type=positive_int, default=2)
    parser.add_argument("--max-retries", type=positive_int, default=3)
    return parser


def main(argv=None):
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args(argv)

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
