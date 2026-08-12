"""Keep the Gazebo mission vehicle shuttling along the top straight."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
import yaml

from .sim_mission_ground_truth_core import shuttle_local_speed


class SimMovingVehicleController(Node):
    def __init__(self) -> None:
        super().__init__("sim_moving_vehicle_controller")
        default_config = str(
            Path(get_package_share_directory("xycar_map_nav"))
            / "config"
            / "sim_mission_layout.yaml"
        )
        self.declare_parameter("layout_config", default_config)
        config_path = Path(
            str(self.get_parameter("layout_config").value)
        ).expanduser()
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        self.world_name = str(config["world"]["name"])
        self.spec = config["moving_vehicle"]
        self.model_name = str(self.spec["name"])
        self.dynamic_pose_index = int(
            config["ground_truth_detection"].get(
                "moving_dynamic_pose_index", 1
            )
        )
        self.speed = float(self.spec["speed_mps"])
        self.local_speed = self.speed
        self.latest_x: float | None = None
        self.publisher = self.create_publisher(
            Twist,
            f"/model/{self.model_name}/cmd_vel",
            10,
        )
        self.create_subscription(
            TFMessage,
            f"/world/{self.world_name}/dynamic_pose/info",
            self._on_dynamic_pose,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f"moving vehicle shuttle: x=[{self.spec['shuttle_min_x']}, "
            f"{self.spec['shuttle_max_x']}] speed={self.speed:.3f} m/s"
        )

    def _on_dynamic_pose(self, message: TFMessage) -> None:
        named_transform = None
        for transform in message.transforms:
            names = (
                str(transform.header.frame_id).strip("/"),
                str(transform.child_frame_id).strip("/"),
            )
            if not any(
                name == self.model_name
                or name.endswith("/" + self.model_name)
                for name in names
            ):
                continue
            named_transform = transform
            break
        if named_transform is not None:
            self.latest_x = float(named_transform.transform.translation.x)
        elif 0 <= self.dynamic_pose_index < len(message.transforms):
            transform = message.transforms[self.dynamic_pose_index]
            self.latest_x = float(transform.transform.translation.x)

    def _publish(self) -> None:
        if self.latest_x is not None:
            self.local_speed = shuttle_local_speed(
                world_x=self.latest_x,
                minimum_x=float(self.spec["shuttle_min_x"]),
                maximum_x=float(self.spec["shuttle_max_x"]),
                current_local_speed=self.local_speed,
                speed_mps=self.speed,
            )
        message = Twist()
        message.linear.x = self.local_speed
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimMovingVehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
