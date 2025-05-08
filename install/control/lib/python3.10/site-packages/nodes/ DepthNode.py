import rclpy
from rclpy.node import Node
import grpc
import data_contracts.depth_pb2_grpc as depth__pb2_grpc
import data_contracts.depth_pb2 as depth__pb2
from core.Dispatcher import Dispatcher
from grpc import FutureTimeoutError

class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_sender_node')
        self.grpc_connected = False
        self.channel = None
        self.stub = None
        self.depth = Dispatcher.getDepth()

        # Retry connection every 2 seconds (non-blocking)
        self.grpc_timer = self.create_timer(2.0, self.connect_to_grpc)

        # Send IMU data every 0.01s
        self.timer = self.create_timer(0.01, self.run)

    def connect_to_grpc(self):
        if self.grpc_connected:
            return

        try:
            self.get_logger().info("Attempting gRPC connection...")
            self.channel = grpc.insecure_channel('localhost:50051')
            grpc.channel_ready_future(self.channel).result(timeout=2)
            self.stub = depth__pb2_grpc.DepthServiceStub(self.channel)
            self.grpc_connected = True
            self.get_logger().info("gRPC connected.")
        except FutureTimeoutError:
            self.get_logger().warn("gRPC connection failed. Will retry...")

    def run(self):
        if not self.grpc_connected:
            return

        depth= self.depth.getDepth()
        pressure = self.depth.getPressure()
        request = depth__pb2.DepthRequest(depth,pressure)
        try:
            response = self.stub.SetDepth(request)
        except grpc.RpcError as e:
            self.grpc_connected = False  # Force reconnection
            self.get_logger().error(f"gRPC failed: {e.details() if hasattr(e, 'details') else str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
