import rclpy
from rclpy.node import Node
import grpc
import threading
from concurrent.futures import TimeoutError as GRPCTimeoutError

from data_contracts import joy_pb2, joy_pb2_grpc
from utils.Configurator import Configurator
from msgs.msg import Joystick

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_channel_node')
        self.mapped_buttons = {}
        self.lock = threading.Lock() 
        # Load configuration
        config = Configurator()

        # gRPC setup
        self.channel = None
        self.stub = None
        self.grpc_connected = False

        # Subscribe to joystick topic
        self.subscriber = self.create_subscription(
            Joystick,
            '/joystick',
            self.joystick_callback,
            10
        )

        self.grpc_timer = self.create_timer(2.0, self.try_connect_grpc)
        self.send_timer = self.create_timer(0.01, self.send_joystick_data)

    def try_connect_grpc(self):
        if self.grpc_connected:
            return
        try:
            self.get_logger().info("Trying to connect to gRPC...")
            self.channel = grpc.insecure_channel('localhost:50051')
            grpc.channel_ready_future(self.channel).result(timeout=2)
            self.stub = joy_pb2_grpc.JoystickServiceStub(self.channel)
            self.grpc_connected = True
            self.get_logger().info("Connected to gRPC.")
        except grpc.FutureTimeoutError:
            self.get_logger().warn("gRPC connection timeout. Retrying...")

    def send_joystick_data(self):
        if not self.grpc_connected:
            return

        # Safely read shared data
        with self.lock:
            button_data = dict(self.mapped_buttons)  # shallow copy

        self.get_logger().info(str(button_data))
        request = joy_pb2.JoystickRequest(
            buttons=button_data,
            axis_x=0.2, axis_y=-0.1, axis_z=0.5,
            roll=1.2, pitch=-0.6, yaw=0.8
        )
        try:
            response = self.stub.UpdateState(request)
            self.get_logger().info(f"Sent joystick data | Status: {response.status}")
        except grpc.RpcError as e:
            self.grpc_connected = False
            self.get_logger().error(f"gRPC error: {e.details() if hasattr(e, 'details') else str(e)}")

    def joystick_callback(self, msg: Joystick):
        # self.get_logger().info(f"  Buttons: {msg.button_names}")
        # self.get_logger().info(f"  States: {msg.button_states}")
        with self.lock:
            self.mapped_buttons = dict(zip(msg.button_names, msg.button_states))

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
