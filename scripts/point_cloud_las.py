#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import laspy
from datetime import datetime

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('pointcloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/orb_slam2_stereo_node/map_points',
            self.pointcloud_callback,
            10
        )

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data.')
        
        # Collect points (x, y, z)
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        # Save to a LAS file
        self.save_las(points)

    def save_las(self, points):

        # Use a set to store unique points (x, y, z)
        unique_points = set(tuple(p) for p in points)  # Convert each point to a tuple
        
        # Convert the set back to a list for LAS writing
        unique_points = list(unique_points)

        # Create a LasHeader
        header = laspy.LasHeader(point_format=3, version="1.2")
        las = laspy.LasData(header)
        
        # Assign point data
        las.x = [p[0] for p in unique_points]
        las.y = [p[1] for p in unique_points]
        las.z = [p[2] for p in unique_points]

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = f'pointcloud_data_{timestamp}.las'
        
        # Save the LAS file
        las.write(output_file)
        self.get_logger().info(f'Saved {len(unique_points)} unique points to {output_file}.')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
