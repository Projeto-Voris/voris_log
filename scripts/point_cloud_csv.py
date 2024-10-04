#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/orb_slam2_stereo_node/map_points',
            self.pointcloud_callback,
            10
        )
#        self.subscription  # prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data.')
        with open('pointcloud_data.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['x', 'y', 'z'])  # Write header
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                csvwriter.writerow([point[0], point[1], point[2]])  # Write each point

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
