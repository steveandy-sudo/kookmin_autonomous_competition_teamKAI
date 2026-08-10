#!/usr/bin/env python3

import shutil
import subprocess

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class MjpegPassthroughNode(Node):
    def __init__(self):
        # Preserve the deployed node name while the package is renamed.
        super().__init__("xycar_wide_camera")
        self.declare_parameter(
            "device",
            "/dev/v4l/by-id/"
            "usb-HD_USB_Camera_HD_USB_Camera-video-index0",
        )
        self.declare_parameter("frame_id", "wide_camera_optical_frame")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 1024)
        self.declare_parameter("fps", 30)
        self.declare_parameter(
            "topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter("power_line_frequency", 2)
        self.declare_parameter("exposure_dynamic_framerate", 0)

        self.device = str(self.get_parameter("device").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.topic = str(self.get_parameter("topic").value)
        self.power_line_frequency = int(
            self.get_parameter("power_line_frequency").value
        )
        self.exposure_dynamic_framerate = int(
            self.get_parameter("exposure_dynamic_framerate").value
        )
        self.publisher = self.create_publisher(
            CompressedImage, self.topic, qos_profile_sensor_data
        )
        self.pipeline = None

        self._configure_camera()
        self._start_pipeline()
        self.get_logger().info(
            f"MJPEG camera ready: {self.device}, "
            f"{self.width}x{self.height}@{self.fps}, topic={self.topic}"
        )

    def _v4l2_ctl(self, arguments):
        if shutil.which("v4l2-ctl") is None:
            self.get_logger().warning("v4l2-ctl not installed; controls skipped")
            return
        result = subprocess.run(
            ["v4l2-ctl", "-d", self.device, *arguments],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if result.returncode != 0:
            self.get_logger().warning(
                f"v4l2-ctl failed: {' '.join(arguments)} | "
                f"{result.stderr.strip()}"
            )

    def _configure_camera(self):
        self._v4l2_ctl(
            [
                "--set-fmt-video="
                f"width={self.width},height={self.height},pixelformat=MJPG",
                f"--set-parm={self.fps}",
            ]
        )
        self._v4l2_ctl(
            ["-c", f"power_line_frequency={self.power_line_frequency}"]
        )
        self._v4l2_ctl(
            [
                "-c",
                "exposure_dynamic_framerate="
                f"{self.exposure_dynamic_framerate}",
            ]
        )

    def _start_pipeline(self):
        Gst.init(None)
        description = (
            f"v4l2src device={self.device} do-timestamp=true ! "
            f"image/jpeg,width={self.width},height={self.height},"
            f"framerate={self.fps}/1 ! "
            "queue max-size-buffers=1 leaky=downstream ! "
            "appsink name=sink emit-signals=true sync=false "
            "max-buffers=1 drop=true"
        )
        self.get_logger().info(f"GStreamer pipeline: {description}")
        self.pipeline = Gst.parse_launch(description)
        sink = self.pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_sample)
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("failed to start camera pipeline")

    def _on_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.format = "jpeg"
        message.data = buffer.extract_dup(0, buffer.get_size())
        self.publisher.publish(message)
        return Gst.FlowReturn.OK

    def destroy_node(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MjpegPassthroughNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
