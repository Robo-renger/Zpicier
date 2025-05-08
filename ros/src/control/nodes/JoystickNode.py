import rclpy
from rclpy.node import Node
import grpc
import threading
from concurrent.futures import TimeoutError as GRPCTimeoutError

from data_contracts import joy_pb2, joy_pb2_grpc
from msgs.msg import Joystick
import time
class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_channel_node')
        self.mapped_buttons = {}
        self.lock = threading.Lock()

        # gRPC setup
        self.channel = None
        self.stub = None
        self.grpc_connected = False

        self.subscriber = self.create_subscription(
            Joystick,
            '/joystick',
            self.joystick_callback,
            10
        )

        self.grpc_timer = self.create_timer(2.0, self.try_connect_grpc)
        self.send_timer = self.create_timer(0.01, self.send_joystick_data)

        # Register shutdown callback

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

        with self.lock:
            button_data = dict(self.mapped_buttons)

        request = joy_pb2.JoystickRequest(
            buttons=button_data,
            axis_x=0.0, axis_y= 1.0,
            roll=0.0, pitch=0.0, yaw=0.0
        )
        try:
            response = self.stub.UpdateState(request)
            # self.get_logger().info(f"Sent joystick data | Status: {response.status}")
        except grpc.RpcError as e:
            self.grpc_connected = False
            self.get_logger().error(f"gRPC error: {e.details() if hasattr(e, 'details') else str(e)}")

    def joystick_callback(self, msg: Joystick):
        with self.lock:
            self.mapped_buttons = dict(zip(msg.button_names, msg.button_states))

    def send_zeroed_message(self):
        if not self.grpc_connected:
            return

        with self.lock:
            zeroed_buttons = {key: False for key, value in self.mapped_buttons.items() if value}

        request = joy_pb2.JoystickRequest(
            buttons=zeroed_buttons,
            axis_x=0.0, axis_y=0.0, axis_z=0.0,
            roll=0.0, pitch=0.0, yaw=0.0
        )
        try:
            response = self.stub.UpdateState(request)
            self.get_logger().info("Sent final zeroed joystick state before shutdown.")
            time.sleep(0.1)
        except grpc.RpcError as e:
            self.get_logger().error(f"Failed to send shutdown message: {e.details() if hasattr(e, 'details') else str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected. Shutting down.")
    finally:
        node.send_zeroed_message()  # ✅ Send final data before shutdown
        node.destroy_node()
        rclpy.shutdown()
