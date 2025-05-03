import rclpy
from rclpy.node import Node

import grpc

import data_contracts.pwm_pb2_grpc as pwm__pb2_grpc
import data_contracts.pwm_pb2 as pwm__pb2

class PCASenderNode(Node):
    def __init__(self):
        super().__init__('pwm_sender_node')
        
        # Setup gRPC client
        self.channel = grpc.insecure_channel('localhost:50051')
        self.stub = pwm__pb2_grpc.PWMServiceStub(self.channel)

        # Send PWM to channels 2 and 4
        self.send_pwm(2, 1500)
        self.send_pwm(4, 1800)

    def send_pwm(self, channel, microseconds):
        request = pwm__pb2.PWMRequest(channel_id=channel, speed=microseconds)
        try:
            response = self.stub.SetPWM(request)
            self.get_logger().info(f"Channel {channel} set to {microseconds} µs. Response: {response.status}")
        except grpc.RpcError as e:
            self.get_logger().error(f"Failed to send PWM: {e.details()}")

def main(args=None):
    rclpy.init(args=args)
    node = PCASenderNode()
    rclpy.shutdown()
