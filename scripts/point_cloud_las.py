#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import laspy

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
        # Create a LasHeader
        header = laspy.LasHeader(point_format=3, version="1.2")
        las = laspy.LasData(header)
        
        # Assign point data
        las.x = [p[0] for p in points]
        las.y = [p[1] for p in points]
        las.z = [p[2] for p in points]
        
        # Save the LAS file
        las.write('pointcloud_data.las')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
