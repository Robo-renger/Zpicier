import rclpy
from rclpy.node import Node

class Logger:
    COLOR_RESET = "\033[0m"
    COLOR_INFO = "\033[92m"    # Green
    COLOR_WARN = "\033[93m"    # Yellow
    COLOR_ERROR = "\033[91m"   # Red
    COLOR_DEBUG = "\033[94m"   # Blue

    def __init__(self, node: Node = None):
        self.node = node
        if node != None:
            self.logger = node.get_logger()

    def logInfoInPlace(self, msg):
        if rclpy.ok() and self.node != None:
            self.logger.info(msg)
        else:
            print(f"{self.COLOR_INFO}[INFO] {msg}{self.COLOR_RESET}")

    def logWarnInPlace(self, msg):
        if  rclpy.ok() and self.node != None:
            self.logger.warn(msg)
        else:
            print(f"{self.COLOR_WARN}[WARN] {msg}{self.COLOR_RESET}")

    def warning(self, msg):
        self.logWarnInPlace(msg)

    def logErrorInPlace(self, msg):
        if  rclpy.ok() and self.node != None:
            self.logger.error(msg)
        else:
            print(f"{self.COLOR_ERROR}[ERROR] {msg}{self.COLOR_RESET}")

    def debug(self, msg):
        if  rclpy.ok() and self.node != None:
            self.logger.debug(msg)
        else:
            print(f"{self.COLOR_DEBUG}[DEBUG] {msg}{self.COLOR_RESET}")
