#!/usr/bin/env python3

import subprocess
import time
import rclpy
from rclpy.node import Node
from utils.EnvParams import EnvParams
from utils.Logger import Logger
class GUIStreamingNode(Node):
    def __init__(self):
        super().__init__('gui_streamer')
        self.logger = Logger(self)
        self.logger.logInfoInPlace("GUI Streaming Node initialized.")
        self.process = None
    def stream(self):
        command = f'mjpg-streamer -o "output_http.so -p 8080 -w {EnvParams().WEB_INDEX_LOCATION}"'
        self.logger.logInfoInPlace(f"Running command: {command}")
        self.process = subprocess.Popen(command, shell=True)

        try:
            while self.process.poll() is None:
                time.sleep(1)  # Sleep in small intervals to allow KeyboardInterrupt
        except KeyboardInterrupt:
            self.logger.logInfoInPlace("KeyboardInterrupt received. Terminating subprocess.")
            self.process.terminate()
            self.process.wait()
        finally:
            self.logger.logInfoInPlace("Subprocess closed.")

def main(args=None):
    rclpy.init(args=args)
    gui_streamer = GUIStreamingNode()

    try:
        gui_streamer.stream()
    except Exception as e:
        gui_streamer.logger.logErrorInPlace(f"Error during streaming: {e}")
    finally:
        gui_streamer.logger.logInfoInPlace("Shutting down ROS node.")
        gui_streamer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
