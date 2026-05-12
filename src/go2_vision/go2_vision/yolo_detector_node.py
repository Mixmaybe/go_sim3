#!/usr/bin/env python3
import json
import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


TARGET_CLASS_NAMES = {"person", "traffic_light", "apriltag"}
WINDOW_NAME = "YOLO robot camera"


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector_node")

        self.declare_parameter("image_topic", "/robot1/color/image_raw")
        self.declare_parameter("model_path", "models/yolo/city_cv_yolo11n_best.pt")
        self.declare_parameter("inference_period_sec", 2.0)
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter(
            "device",
            "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("publish_annotated_image", True)
        self.declare_parameter("enable_yolo", True)
        self.declare_parameter("show_yolo_window", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.inference_period_sec = max(
            0.05, float(self.get_parameter("inference_period_sec").value)
        )
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.device = str(self.get_parameter("device").value)
        self.publish_annotated_image = bool(
            self.get_parameter("publish_annotated_image").value
        )
        self.enable_yolo = bool(self.get_parameter("enable_yolo").value)
        self.show_yolo_window = bool(self.get_parameter("show_yolo_window").value)
        if self.show_yolo_window and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warning(
                "DISPLAY is not set; disabling YOLO OpenCV window. "
                "Use show_yolo_window:=false for headless runs."
            )
            self.show_yolo_window = False

        self.bridge = CvBridge() if CvBridge is not None else None
        self.model = None
        self.model_names: Dict[int, str] = {}
        self.last_inference_time = 0.0
        self.last_detections: List[Dict[str, Any]] = []
        self.last_annotated_frame = None
        self._missing_cv_bridge_logged = False
        self._missing_cv2_logged = False
        self._missing_ultralytics_logged = False

        self.annotated_pub = self.create_publisher(
            Image, "/robot1/vision/annotated_image", 10
        )
        self.log_pub = self.create_publisher(String, "/robot1/vision/yolo_log", 10)
        self.detections_pub = self.create_publisher(
            String, "/robot1/vision/detections", 10
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.enable_service = self.create_service(
            SetBool, "/robot1/vision/set_yolo_enabled", self.set_yolo_enabled
        )

        if self.enable_yolo:
            self.publish_log("YOLO: включен")
        else:
            self.publish_log("YOLO: выключен")

        self.get_logger().info(
            "YOLO detector ready: image_topic=%s, model_path=%s, period=%.2fs, enabled=%s"
            % (
                self.image_topic,
                self.model_path,
                self.inference_period_sec,
                self.enable_yolo,
            )
        )

    def destroy_node(self):
        if cv2 is not None and self.show_yolo_window:
            try:
                cv2.destroyWindow(WINDOW_NAME)
            except Exception:
                pass
        super().destroy_node()

    def set_yolo_enabled(self, request: SetBool.Request, response: SetBool.Response):
        requested_state = bool(request.data)
        if self.enable_yolo == requested_state:
            state_text = "включен" if requested_state else "выключен"
            self.publish_log(f"YOLO: {state_text}")
            response.success = True
            response.message = f"YOLO уже {state_text}"
            return response

        self.enable_yolo = requested_state
        self.last_inference_time = 0.0
        if not self.enable_yolo:
            self.last_detections = []

        log_line = "YOLO: включен" if self.enable_yolo else "YOLO: выключен"
        self.publish_log(log_line)
        response.success = True
        response.message = log_line
        return response

    def image_callback(self, msg: Image):
        if self.bridge is None:
            if not self._missing_cv_bridge_logged:
                self.get_logger().error("cv_bridge is not available.")
                self._missing_cv_bridge_logged = True
            return
        if cv2 is None:
            if not self._missing_cv2_logged:
                self.get_logger().error(
                    "OpenCV is not available. Install it with: pip install opencv-python"
                )
                self._missing_cv2_logged = True
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Failed to convert camera frame: {exc}")
            return

        annotated = frame.copy()
        now = time.monotonic()
        should_infer = (
            self.enable_yolo
            and now - self.last_inference_time >= self.inference_period_sec
        )

        if should_infer:
            self.last_inference_time = now
            if self.load_model():
                detections = self.run_inference(frame)
                self.last_detections = detections
                self.publish_detections(detections)
                self.publish_detection_logs(detections)

        self.draw_detections(annotated, self.last_detections)
        self.draw_status(annotated)
        self.last_annotated_frame = annotated

        if self.publish_annotated_image:
            self.publish_annotated(msg, annotated)

        if self.show_yolo_window:
            self.show_window(annotated)

    def load_model(self) -> bool:
        if self.model is not None:
            return True

        try:
            from ultralytics import YOLO
        except ImportError:
            if not self._missing_ultralytics_logged:
                self.get_logger().error(
                    "Ultralytics is not installed. Install it with: pip install ultralytics"
                )
                self.publish_log(
                    "YOLO: Ultralytics is not installed. Install it with: pip install ultralytics"
                )
                self._missing_ultralytics_logged = True
            return False

        resolved_model_path = self.resolve_model_path(self.model_path)
        if not resolved_model_path.exists():
            self.get_logger().error(f"YOLO model not found: {resolved_model_path}")
            self.publish_log(f"YOLO: модель не найдена: {resolved_model_path}")
            return False

        try:
            self.get_logger().info(f"Loading YOLO model: {resolved_model_path}")
            self.model = YOLO(str(resolved_model_path))
            names = getattr(self.model, "names", {}) or {}
            self.model_names = {int(k): str(v) for k, v in names.items()}
            self.get_logger().info("YOLO model loaded.")
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to load YOLO model: {exc}")
            self.publish_log(f"YOLO: ошибка загрузки модели: {exc}")
            return False

    def run_inference(self, frame) -> List[Dict[str, Any]]:
        kwargs = {
            "source": frame,
            "imgsz": self.imgsz,
            "conf": self.confidence_threshold,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device

        try:
            results = self.model.predict(**kwargs)
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            self.publish_log(f"YOLO: ошибка inference: {exc}")
            return []

        if not results:
            return []

        height, width = frame.shape[:2]
        detections = []
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return []

        for box in boxes:
            parsed = self.parse_box(box, width, height)
            if parsed is not None:
                detections.append(parsed)

        return detections

    def parse_box(self, box, width: int, height: int) -> Optional[Dict[str, Any]]:
        try:
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            xyxy = box.xyxy[0].detach().cpu().tolist()
        except Exception:
            return None

        if confidence < self.confidence_threshold:
            return None

        class_name = self.normalize_class_name(
            self.model_names.get(class_id, str(class_id))
        )
        if class_name not in TARGET_CLASS_NAMES:
            return None

        x1, y1, x2, y2 = self.clip_bbox(xyxy, width, height)
        center_x = (x1 + x2) / 2.0
        zone = self.zone_for_x(center_x, width)

        return {
            "class_id": class_id,
            "class_name": class_name,
            "confidence": round(confidence, 4),
            "bbox": [x1, y1, x2, y2],
            "zone": zone,
        }

    def publish_annotated(self, original_msg: Image, annotated):
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = original_msg.header
            self.annotated_pub.publish(out_msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to publish annotated image: {exc}")

    def publish_detections(self, detections: Sequence[Dict[str, Any]]):
        msg = String()
        msg.data = json.dumps(list(detections), ensure_ascii=False)
        self.detections_pub.publish(msg)

    def publish_detection_logs(self, detections: Sequence[Dict[str, Any]]):
        if not detections:
            self.publish_log("YOLO: объектов не обнаружено")
            return

        for detection in detections:
            self.publish_log(
                "YOLO: распознан %s, confidence=%.2f, zone=%s"
                % (
                    detection["class_name"],
                    float(detection["confidence"]),
                    detection["zone"],
                )
            )

    def publish_log(self, text: str):
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)
        self.get_logger().info(text)

    def draw_detections(self, frame, detections: Sequence[Dict[str, Any]]):
        for detection in detections:
            x1, y1, x2, y2 = detection["bbox"]
            label = "%s %.2f" % (
                detection["class_name"],
                float(detection["confidence"]),
            )
            cv2.rectangle(frame, (x1, y1), (x2, y2), (30, 220, 80), 2)
            self.draw_label(frame, label, x1, y1)

    def draw_status(self, frame):
        status = "YOLO ON" if self.enable_yolo else "YOLO OFF"
        text = f"{status} | period {self.inference_period_sec:.1f}s"
        cv2.rectangle(frame, (8, 8), (260, 42), (0, 0, 0), -1)
        cv2.putText(
            frame,
            text,
            (16, 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def draw_label(self, frame, label: str, x: int, y: int):
        baseline = 0
        (label_width, label_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        )
        top = max(0, y - label_height - baseline - 6)
        cv2.rectangle(
            frame,
            (x, top),
            (x + label_width + 8, top + label_height + baseline + 6),
            (30, 220, 80),
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x + 4, top + label_height + 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    def show_window(self, frame):
        try:
            cv2.imshow(WINDOW_NAME, frame)
            cv2.waitKey(1)
        except Exception as exc:
            self.get_logger().warning(
                f"Disabling YOLO OpenCV window because imshow failed: {exc}"
            )
            self.show_yolo_window = False

    @staticmethod
    def normalize_class_name(name: str) -> str:
        return name.strip().lower().replace(" ", "_").replace("-", "_")

    @staticmethod
    def clip_bbox(
        bbox: Sequence[float], width: int, height: int
    ) -> Tuple[int, int, int, int]:
        x1, y1, x2, y2 = bbox
        x1 = max(0, min(width - 1, int(round(x1))))
        y1 = max(0, min(height - 1, int(round(y1))))
        x2 = max(0, min(width - 1, int(round(x2))))
        y2 = max(0, min(height - 1, int(round(y2))))
        return x1, y1, x2, y2

    @staticmethod
    def zone_for_x(center_x: float, width: int) -> str:
        if center_x < width / 3.0:
            return "left"
        if center_x > 2.0 * width / 3.0:
            return "right"
        return "center"

    @staticmethod
    def resolve_model_path(model_path: str) -> Path:
        path = Path(model_path).expanduser()
        if path.is_absolute():
            return path

        cwd_path = Path.cwd() / path
        if cwd_path.exists():
            return cwd_path

        for parent in Path(__file__).resolve().parents:
            candidate = parent / path
            if candidate.exists():
                return candidate

        return cwd_path


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
