#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from std_msgs.msg import Bool

from cone_il.cone_ai_driver_node import ConeAIDriver


class SwitchableConeAIDriver(ConeAIDriver):
    def __init__(self):
        super().__init__()

        self.declare_parameter('enable_topic', '/cone_ai/enable')
        self.declare_parameter('start_enabled', False)
        self.enabled = bool(self.get_parameter('start_enabled').value)

        enable_topic = str(self.get_parameter('enable_topic').value)
        self.create_subscription(Bool, enable_topic, self.enable_callback, 10)
        self.get_logger().info(f'AI direct enable topic={enable_topic}, start_enabled={self.enabled}')

    def enable_callback(self, msg: Bool):
        enabled = bool(msg.data)
        if self.enabled != enabled:
            self._log_status_once('enabled' if enabled else 'disabled')
        self.enabled = enabled

    def control_once(self):
        if not self.enabled:
            self._log_status_once('disabled')
            return
        super().control_once()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SwitchableConeAIDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.drive(0.0, 0.0)
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
