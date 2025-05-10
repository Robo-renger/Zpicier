import rclpy
from rclpy.node import Node
import grpc
import time

import data_contracts.pwm_pb2_grpc as pwm__pb2_grpc
import data_contracts.pwm_pb2 as pwm__pb2
from grpc import FutureTimeoutError
from core.Dispatcher import Dispatcher
from utils.Logger import Logger
class PWMClientNode(Node):
    def __init__(self):
        super().__init__('pwm_client_node')
        self.grpc_connected = False
        self.channel = None
        self.stub = None
        self.logger = Logger(self)
        self.PCA = Dispatcher.getPWMDriver()
        # Try connecting every 2 seconds until successful
        self.grpc_timer = self.create_timer(2.0, self.connect_to_grpc)

        # Poll for PWM values every 0.5s
        self.timer = self.create_timer(0.01, self.get_pwm)

    def connect_to_grpc(self):
        if self.grpc_connected:
            return

        try:
            self.logger.logInfoInPlace("Attempting gRPC connection...")
            self.channel = grpc.insecure_channel('localhost:50051')
            grpc.channel_ready_future(self.channel).result(timeout=2)

            self.stub = pwm__pb2_grpc.PWMServiceStub(self.channel)
            self.grpc_connected = True
            self.logger.logInfoInPlace("gRPC connected.")

        except FutureTimeoutError:
            self.logger.logWarnInPlace("gRPC connection failed. Will retry...")

    def get_pwm(self):
        if not self.grpc_connected or self.stub is None:
            # self.logger.logWarnInPlace("Stub not ready — skipping PWM fetch.")
            return
        try:
            request = pwm__pb2.GetPWMRequest()
            response = self.stub.GetPWM(request)

            for entry in response.values:
                self.PCA.PWMWrite(entry.channel, entry.microseconds)
                # self.logger.logInfoInPlace(f'PWM - Channel: {entry.channel} → {entry.microseconds:.2f} µs')

        except grpc.RpcError as e:
            self.logger.logErrorInPlace(f"gRPC failed: {e.details() if hasattr(e, 'details') else str(e)}")
            self.grpc_connected = False  # Force reconnection
            self.stub = None


def main(args=None):
    rclpy.init(args=args)
    node = PWMClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.logInfoInPlace("KeyboardInterrupt received. Shutting down PWM client node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
   

if __name__ == '__main__':
    main()
