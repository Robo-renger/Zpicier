import rclpy
from rclpy.node import Node

class Logger:
    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()

    def logInfoInPlace(self, msg):
        if rclpy.ok():
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def logWarnInPlace(self, msg):
        if rclpy.ok():
            self.logger.warn(msg)
        else:
            print(f"[WARN] {msg}")

    def warning(self, msg):
        self.logWarnInPlace(msg)

    def logErrorInPlace(self, msg):
        if rclpy.ok():
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def debug(self, msg):
        if rclpy.ok():
            self.logger.debug(msg)
        else:
            print(f"[DEBUG] {msg}")
