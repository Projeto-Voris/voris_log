#!/usr/bin/env python3
from multiprocessing.util import get_logger

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import numpy as np
import cv2
from noise import pnoise2
from screeninfo import get_monitors
import random

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.image_width = 800
        self.image_height = 600
        self.window_position = (0, 0)

        # Get node parameters
        self.declare_parameter('frequency', 3.0)
        self.declare_parameter('monitor_name', 'DP-0')

        # Set parameters inside class
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.get_screen_resolution(self.get_parameter('monitor_name').get_parameter_value().string_value)

        # Construct the first image
        self.image = self.generate_noise_image()

        # Create window to project
        self.window_name = 'Random Image'
        self.construct_window()

        # Create the service
        self.change_image_service = self.create_service(SetBool, 'pattern_change', self.change_image_cb)

        # Main loop to display the image
        self.timer = self.create_timer(0.01, self.timer_image_cb)  # Update every 10 ms for smoother display

    def get_screen_resolution(self, monitor_name):
        for monitor in get_monitors():
            if monitor.name == monitor_name:
                self.get_logger().info(f'Screen resolution: {monitor.width}x{monitor.height}')
                self.get_logger().info(f'Screen position: {monitor.x}x{monitor.y}')
                self.image_width = monitor.width
                self.image_height = monitor.height
                self.window_position = (monitor.x, monitor.y)

    def generate_noise_image(self):
        # Prepare grid coordinates
        i, j = np.meshgrid(np.arange(self.image_height), np.arange(self.image_width), indexing='ij')

        # Generate a new random base seed
        random_base = random.randint(30, 50)

        # Vectorized function to compute noise for each coordinate
        noise_function = np.vectorize(lambda x, y: pnoise2(x / self.frequency, y / self.frequency, octaves=8,
                                                           persistence=0.2, lacunarity=2.0,
                                                           repeatx=self.image_width, repeaty=self.image_height,
                                                           base=random_base))

        # Apply noise function to the entire grid
        noise_values = noise_function(i, j)

        # Normalize and convert to uint8
        self.get_logger().info("Image created")
        pattern_image = np.clip((noise_values + 0.35) * 255, 0, 255).astype(np.uint8)
        return pattern_image

    def construct_window(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow(self.window_name, self.window_position[0], self.window_position[1])

    def timer_image_cb(self):
        cv2.imshow(self.window_name, self.image)
        cv2.waitKey(10)  # Lower wait time to reduce latency

    def change_image_cb(self, request, response):
        self.get_logger().info("Changing projected image")
        self.image = self.generate_noise_image()
        cv2.imshow(self.window_name, self.image)  # Force refresh
        self.get_logger().info("Image changed")
        response.success = True
        response.message = "Image successfully changed."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
