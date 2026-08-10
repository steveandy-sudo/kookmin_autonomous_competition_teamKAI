"""Publish deterministic semantic boxes for the Gazebo mission objects."""

from __future__ import annotations

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from my_rule_msgs.msg import ObjectDetection
from my_rule_msgs.msg import ObjectDetectionArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
import yaml

from .sim_mission_ground_truth_core import EgoPose
from .sim_mission_ground_truth_core import MissionTarget
from .sim_mission_ground_truth_core import project_target
from .sim_mission_ground_truth_core import quaternion_to_yaw


CLASS_IDS = {
    "car": 0,
    "cone": 1,
    "red": 2,
    "yellow": 3,
    "green": 4,
}


def _cone_positions(spec: dict) -> list[tuple[str, float, float]]:
    start = float(spec["start_y"])
    end = float(spec["end_y"])
    spacing = float(spec["spacing_m"])
    count = int(math.floor((start - end) / spacing)) + 1
    result = []
    for index in range(count):
        y = start - index * spacing
        result.extend(
            (
                (
                    f"{spec['name_prefix']}_east_{index + 1:02d}",
                    float(spec["east_x"]),
                    y,
                ),
                (
                    f"{spec['name_prefix']}_west_{index + 1:02d}",
                    float(spec["west_x"]),
                    y,
                ),
            )
        )
    return result


class SimMissionGroundTruth(Node):
    """Convert known world poses to integration detector messages."""

    def __init__(self) -> None:
        super().__init__("sim_mission_ground_truth")
        default_config = str(
            Path(get_package_share_directory("my_drive"))
            / "config"
            / "sim_mission_layout.yaml"
        )
        self.declare_parameter("layout_config", default_config)
        self.declare_parameter(
            "detections_topic", "/my_rule/sim_ground_truth_detections"
        )
        config_path = Path(
            str(self.get_parameter("layout_config").value)
        ).expanduser()
        self.config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        detection = self.config["ground_truth_detection"]
        self.image_width = int(detection["image_width"])
        self.image_height = int(detection["image_height"])
        self.horizontal_fov = float(detection["horizontal_fov_rad"])
        self.ego_dynamic_pose_index = int(
            detection.get("ego_dynamic_pose_index", 0)
        )
        self.moving_dynamic_pose_index = int(
            detection.get("moving_dynamic_pose_index", 1)
        )
        self.maximum_ranges = {
            "car": float(detection["car_max_range_m"]),
            "cone": float(detection["cone_max_range_m"]),
            "signal": float(detection["signal_max_range_m"]),
        }
        self.ego_pose: EgoPose | None = None
        self.have_world_ego_pose = False
        self.moving_poses: dict[str, tuple[float, float]] = {}
        self.targets = self._configured_targets()
        self.publisher = self.create_publisher(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            "/model/xycar_ackermann/odometry",
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TFMessage,
            f"/world/{self.config['world']['name']}/dynamic_pose/info",
            self._on_dynamic_pose,
            qos_profile_sensor_data,
        )
        self.create_timer(
            1.0 / max(float(detection["publish_hz"]), 1.0),
            self._publish,
        )
        self.get_logger().info(
            "ground-truth mission detector ready: "
            f"{len(self.targets)} objects, "
            f"config={config_path}"
        )

    def _configured_targets(self) -> list[MissionTarget]:
        targets = []
        moving = self.config["moving_vehicle"]
        targets.append(self._vehicle_target(moving))
        targets.extend(
            self._vehicle_target(item)
            for item in self.config.get("fixed_vehicles", [])
        )
        signal = self.config["traffic_light"]
        pose = tuple(float(value) for value in signal["pose"])
        offset = tuple(
            float(value)
            for value in signal.get("detection_offset_xy", (0.0, 0.58))
        )
        cosine, sine = math.cos(pose[5]), math.sin(pose[5])
        targets.append(
            MissionTarget(
                name=str(signal["name"]),
                class_name=str(signal.get("class_name", "green")),
                x=pose[0] + cosine * offset[0] - sine * offset[1],
                y=pose[1] + sine * offset[0] + cosine * offset[1],
                width_m=float(signal.get("head_width_m", 0.50)),
                height_m=float(signal.get("head_height_m", 0.16)),
                elevated=True,
            )
        )
        cone = self.config["cone_corridor"]
        targets.extend(
            MissionTarget(
                name=name,
                class_name=str(cone.get("class_name", "cone")),
                x=x,
                y=y,
                width_m=2.0 * float(cone["base_radius_m"]),
                height_m=float(cone["height_m"]),
            )
            for name, x, y in _cone_positions(cone)
        )
        return targets

    @staticmethod
    def _vehicle_target(spec: dict) -> MissionTarget:
        pose = tuple(float(value) for value in spec["pose"])
        size = tuple(float(value) for value in spec["size"])
        return MissionTarget(
            name=str(spec["name"]),
            class_name=str(spec.get("class_name", "car")),
            x=pose[0],
            y=pose[1],
            width_m=size[1],
            height_m=size[2],
        )

    def _on_odometry(self, message: Odometry) -> None:
        if self.have_world_ego_pose:
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.ego_pose = EgoPose(
            x=float(position.x),
            y=float(position.y),
            yaw=quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )

    def _on_dynamic_pose(self, message: TFMessage) -> None:
        moving_name = str(self.config["moving_vehicle"]["name"])
        named_moving_pose = None
        for transform in message.transforms:
            names = (
                str(transform.header.frame_id).strip("/"),
                str(transform.child_frame_id).strip("/"),
            )
            if not any(
                name == moving_name or name.endswith("/" + moving_name)
                for name in names
            ):
                continue
            named_moving_pose = transform
            break

        transforms = message.transforms
        if 0 <= self.ego_dynamic_pose_index < len(transforms):
            ego = transforms[self.ego_dynamic_pose_index].transform
            self.ego_pose = EgoPose(
                x=float(ego.translation.x),
                y=float(ego.translation.y),
                yaw=quaternion_to_yaw(
                    float(ego.rotation.x),
                    float(ego.rotation.y),
                    float(ego.rotation.z),
                    float(ego.rotation.w),
                ),
            )
            self.have_world_ego_pose = True

        moving_transform = named_moving_pose
        if (
            moving_transform is None
            and 0 <= self.moving_dynamic_pose_index < len(transforms)
        ):
            moving_transform = transforms[self.moving_dynamic_pose_index]
        if moving_transform is not None:
            translation = moving_transform.transform.translation
            self.moving_poses[moving_name] = (
                float(translation.x),
                float(translation.y),
            )

    def _publish(self) -> None:
        if self.ego_pose is None:
            return
        message = ObjectDetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "xycar_ackermann/chassis/front_camera"
        message.image_width = self.image_width
        message.image_height = self.image_height
        moving_name = str(self.config["moving_vehicle"]["name"])
        for configured in self.targets:
            target = configured
            if (
                configured.name == moving_name
                and moving_name in self.moving_poses
            ):
                x, y = self.moving_poses[moving_name]
                target = MissionTarget(
                    name=configured.name,
                    class_name=configured.class_name,
                    x=x,
                    y=y,
                    width_m=configured.width_m,
                    height_m=configured.height_m,
                    elevated=configured.elevated,
                )
            category = (
                "signal"
                if target.class_name in {"red", "yellow", "green"}
                else target.class_name
            )
            box = project_target(
                self.ego_pose,
                target,
                image_width=self.image_width,
                image_height=self.image_height,
                horizontal_fov_rad=self.horizontal_fov,
                maximum_range_m=self.maximum_ranges.get(category, 4.0),
            )
            if box is None:
                continue
            item = ObjectDetection()
            item.class_name = target.class_name
            item.class_id = CLASS_IDS.get(target.class_name, -1)
            item.confidence = 0.99
            item.xmin = box.xmin
            item.ymin = box.ymin
            item.xmax = box.xmax
            item.ymax = box.ymax
            message.detections.append(item)
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimMissionGroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
