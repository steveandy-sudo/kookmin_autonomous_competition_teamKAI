import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


class XycarSensorRvizRepublisher(Node):
    def __init__(self):
        super().__init__("xycar_sensor_rviz_republisher")

        self.declare_parameter("input_scan_topic", "/scan")
        self.declare_parameter("output_scan_topic", "/rviz/scan")
        self.declare_parameter("laser_frame_id", "laser_frame")
        self.declare_parameter("input_image_topic", "/image_raw")
        self.declare_parameter("output_image_topic", "/rviz/image_raw")
        self.declare_parameter("input_camera_info_topic", "/camera_info")
        self.declare_parameter("output_camera_info_topic", "/rviz/camera_info")
        self.declare_parameter("camera_frame_id", "camera_frame")

        self.laser_frame_id = self.get_parameter("laser_frame_id").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        rviz_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter("output_scan_topic").value,
            rviz_qos,
        )
        self.image_pub = self.create_publisher(
            Image,
            self.get_parameter("output_image_topic").value,
            rviz_qos,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            self.get_parameter("output_camera_info_topic").value,
            rviz_qos,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("input_scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter("input_image_topic").value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("input_camera_info_topic").value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Republishing Gazebo sensors for RViz: /scan -> /rviz/scan, "
            "/image_raw -> /rviz/image_raw"
        )

    def scan_callback(self, msg):
        msg.header.frame_id = self.laser_frame_id
        self.scan_pub.publish(msg)

    def image_callback(self, msg):
        msg.header.frame_id = self.camera_frame_id
        self.image_pub.publish(msg)

    def camera_info_callback(self, msg):
        msg.header.frame_id = self.camera_frame_id
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XycarSensorRvizRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
