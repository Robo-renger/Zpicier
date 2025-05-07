import rclpy
from rclpy.node import Node
import grpc
import time

from data_contracts import joy_pb2, joy_pb2_grpc
from utils.Configurator import Configurator
class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_channel_node')
        config = Configurator()
        print(config.fetchData(Configurator.BUTTONS))
        self.channel = None
        self.stub = None
        self.grpc_connected = False

        # Attempt initial connection
        self.connect_to_grpc()

        # ROS 2 Timer to periodically send joystick data
        self.timer = self.create_timer(1.0, self.send_fake_joystick_data)

    def connect_to_grpc(self):
        while not self.grpc_connected and rclpy.ok():
            try:
                self.get_logger().info("Attempting gRPC connection...")
                self.channel = grpc.insecure_channel('localhost:50051')
                grpc.channel_ready_future(self.channel).result(timeout=3)
                self.stub = joy_pb2_grpc.JoystickServiceStub(self.channel)
                self.grpc_connected = True
                self.get_logger().info("gRPC connection established.")
            except grpc.FutureTimeoutError:
                self.get_logger().warn("gRPC connection failed, retrying...")
                time.sleep(2)

    def send_fake_joystick_data(self):
        if not self.grpc_connected:
            self.connect_to_grpc()

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
            self.grpc_connected = False
            self.get_logger().error(f"gRPC error: {e.details() if hasattr(e, 'details') else str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
