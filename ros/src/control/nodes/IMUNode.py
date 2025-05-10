import rclpy
from rclpy.node import Node
import grpc
import data_contracts.imu_pb2_grpc as imu__pb2_grpc
import data_contracts.imu_pb2 as imu__pb2
from core.Dispatcher import Dispatcher
from grpc import FutureTimeoutError
from utils.Logger import Logger
class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_sender_node')
        self.grpc_connected = False
        self.channel = None
        self.stub = None
        self.imu = Dispatcher.getIMU()
        self.logger = Logger(self)
        # Retry connection every 2 seconds (non-blocking)
        self.grpc_timer = self.create_timer(2.0, self.connect_to_grpc)

        # Send IMU data every 0.01s
        self.timer = self.create_timer(0.01, self.run)

    def connect_to_grpc(self):
        if self.grpc_connected:
            return

        try:
            self.logger.logInfoInPlace("Attempting gRPC connection...")
            self.channel = grpc.insecure_channel('localhost:50051')
            grpc.channel_ready_future(self.channel).result(timeout=2)
            self.stub = imu__pb2_grpc.IMUServiceStub(self.channel)
            self.grpc_connected = True
            self.logger.logInfoInPlace("gRPC connected.")
        except FutureTimeoutError:
            self.logger.logWarnInPlace("gRPC connection failed. Will retry...")

    def run(self):
        if not self.grpc_connected:
            return

        roll, pitch, yaw = self.imu.getEulerAngles()
        request = imu__pb2.IMURequest(pitch=pitch, roll=roll, yaw=yaw)
        try:
            response = self.stub.SetEulerAngles(request)
        except grpc.RpcError as e:
            self.grpc_connected = False  # Force reconnection
            self.logger.logErrorInPlace(f"gRPC failed: {e.details() if hasattr(e, 'details') else str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] Ctrl+C detected, shutting down gracefully")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()    
