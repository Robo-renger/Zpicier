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
        self.channel = None
        self.stub = None
        self.grpc_connected = False
        self.depth = Dispatcher.getDepth()

        self.grpc_timer = self.create_timer(2.0, self.connect_to_grpc)
        self.timer = self.create_timer(0.01, self.run)

    def connect_to_grpc(self):
        # Always attempt to reconnect if disconnected
        if self.grpc_connected:
            return

        self.get_logger().info("Attempting gRPC connection...")
        try:
            self.channel = grpc.insecure_channel('localhost:50051')
            grpc.channel_ready_future(self.channel).result(timeout=2)
            self.stub = depth__pb2_grpc.DepthServiceStub(self.channel)
            self.grpc_connected = True
            self.get_logger().info("gRPC connected successfully.")
        except FutureTimeoutError:
            self.get_logger().warn("gRPC connection failed. Will retry...")

    def run(self):
        if not self.grpc_connected:
            return

        try:
            depth = self.depth.getDepth()
            pressure = self.depth.getPressure()
            request = depth__pb2.DepthRequest(depth=depth, pressure=pressure)
            self.stub.SetDepth(request)
        except grpc.RpcError as e:
            self.grpc_connected = False  # Mark disconnected to retry
            self.get_logger().error(f"gRPC error: {e.details() if hasattr(e, 'details') else str(e)}")
        except Exception as ex:
            self.get_logger().error(f"Unexpected error: {str(ex)}")
            self.grpc_connected = False  # Force retry if unexpected error occurs

def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()    
