#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import CameraInfo
import yaml
import sys

class CameraInfoPublisher(Node):
    def __init__(self, yaml_file_left: str, yaml_file_right: str):
        super().__init__('camera_info_publisher')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.declare_parameter('left_frameID', '/SM2/left_camera_link')
        self.declare_parameter('right_frameID', '/SM2/left_camera_link')
        self.publisher_left = self.create_publisher(CameraInfo, 'left/camera_info', qos_profile)
        self.publisher_right = self.create_publisher(CameraInfo, 'right/camera_info', qos_profile)

        self.cam_info_left = self.load_yaml(yaml_file_left)
        self.cam_info_left.header.frame_id = self.get_parameter('left_frameID').value

        self.cam_info_right = self.load_yaml(yaml_file_right)
        self.cam_info_right.header.frame_id = self.get_parameter('right_frameID').value

        # Publish at 1 Hz (adjust as needed)
        self.get_logger().info('Publishing camera info')
        self.timer = self.create_timer(1.0, self.publish_camera_info)

    def publish_camera_info(self):
        now = self.get_clock().now().to_msg()
        self.cam_info_left.header.stamp = now
        self.cam_info_right.header.stamp = now

        self.publisher_left.publish(self.cam_info_left)
        self.publisher_right.publish(self.cam_info_right)

    def load_yaml(self, yaml_file: str) -> CameraInfo:
        with open(yaml_file, 'r') as f:
            calib_data = yaml.safe_load(f)

        cam_info = CameraInfo()
        cam_info.width = calib_data["image_width"]
        cam_info.height = calib_data["image_height"]
        cam_info.distortion_model = calib_data["distortion_model"]

        cam_info.d = calib_data["distortion_coefficients"]["data"]
        cam_info.k = calib_data["camera_matrix"]["data"]
        cam_info.r = calib_data["rectification_matrix"]["data"]
        cam_info.p = calib_data["projection_matrix"]["data"]

        return cam_info


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run my_package camera_info_publisher <yaml_left> <yaml_right>")
        rclpy.shutdown()
        return

    yaml_file_left = sys.argv[1]
    yaml_file_right = sys.argv[2]
    node = CameraInfoPublisher(yaml_file_left, yaml_file_right)
    rclpy.spin(node)  # <-- Add this line

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
