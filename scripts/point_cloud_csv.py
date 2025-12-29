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
            '/pointcloud2',
            self.pointcloud_callback,
            10
        )
#        self.subscription  # prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info('Received point cloud data.')

        # Check if 'rgb' or 'rgba' field exists
        field_names = [field.name for field in msg.fields]

        has_rgb = 'rgb' in field_names
        has_rgba = 'rgba' in field_names

        if has_rgba:
            csv_header = ['x', 'y', 'z', 'r', 'g', 'b', 'a']
            fields = ("x", "y", "z", "rgba")
        elif has_rgb:
            csv_header = ['x', 'y', 'z', 'r', 'g', 'b']
            fields = ("x", "y", "z", "rgb")
        else:
            csv_header = ['x', 'y', 'z']
            fields = ("x", "y", "z")

        with open('pointcloud_data.csv', 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(csv_header)  # Write header

            for point in pc2.read_points(msg, field_names=fields, skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                if has_rgba:
                    rgba = int(point[3])
                    r = (rgba >> 24) & 0xFF
                    g = (rgba >> 16) & 0xFF
                    b = (rgba >> 8) & 0xFF
                    a = rgba & 0xFF
                    csvwriter.writerow([x, y, z, r, g, b, a])
                elif has_rgb:
                    import struct
                    rgb_float = point[3]
                    rgb = struct.unpack('I', struct.pack('f', rgb_float))[0]
                    # self.get_logger().info(f'RGB value: {rgb}')
                    r = (rgb >> 16) & 0xFF
                    g = (rgb >> 8) & 0xFF
                    b = rgb & 0xFF
                    csvwriter.writerow([x, y, z, r, g, b])
                else:
                    csvwriter.writerow([x, y, z])

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
