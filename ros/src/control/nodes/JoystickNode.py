import rclpy
from rclpy.node import Node
import grpc
import time

from data_contracts import joy_pb2, joy_pb2_grpc

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_channel_node')

        # gRPC client
        self.channel = grpc.insecure_channel('localhost:50051')
        self.stub = joy_pb2_grpc.JoystickServiceStub(self.channel)

        # ROS 2 Timer to simulate sending joystick data every second
        self.timer = self.create_timer(1.0, self.send_fake_joystick_data)

    def send_fake_joystick_data(self):
        # Simulated button states
        buttons = {
            "main_light": True,
            "pump": False,
            "valve": True
        }

        # Simulated axis values
        request = joy_pb2.JoystickRequest(
            buttons=buttons,
            axis_x=0.2,
            axis_y=-0.1,
            axis_z=0.5,
            roll=1.2,
            pitch=-0.6,
            yaw=0.8
        )

        try:
            response = self.stub.UpdateState(request)
            self.get_logger().info(f"Sent joystick data | Status: {response.status}")
        except grpc.RpcError as e:
            self.get_logger().error(f"gRPC error: {e.details()}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
