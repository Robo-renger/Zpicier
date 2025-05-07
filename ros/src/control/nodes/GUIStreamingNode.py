#!/usr/bin/env python3

import subprocess
import rclpy
from rclpy.node import Node
from utils.EnvParams import EnvParams

class GUIStreamingNode(Node):
    def __init__(self):
        super().__init__('gui_streamer')
        self.get_logger().info("GUI Streaming Node initialized.")

    def stream(self):
        command = f'mjpg-streamer -o "output_http.so -p 8080 -w {EnvParams().WEB_INDEX_LOCATION}"'
        self.get_logger().info(f"Running command: {command}")
        subprocess.run(command, shell=True)

def main(args=None):
    rclpy.init(args=args)
    gui_streamer = GUIStreamingNode()
    gui_streamer.stream()
    rclpy.spin(gui_streamer)

    gui_streamer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
