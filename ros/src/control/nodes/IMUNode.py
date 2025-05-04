import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import grpc
import data_contracts.imu_pb2_grpc as imu__pb2_grpc
import data_contracts.imu_pb2 as imu__pb2
from core.Dispatcher import Dispatcher
class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_sender_node')
        self.stub = imu__pb2_grpc.IMUServiceStub(grpc.insecure_channel("localhost:50051"))
        self.imu = Dispatcher.getIMU()

        # Run every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.01, self.run)

    def run(self):
        data = self.imu.getEulerAngles()
        roll, pitch, yaw = data

        request = imu__pb2.IMURequest(pitch=pitch, roll=roll, yaw=yaw)
        try:
            response = self.stub.SetEulerAngles(request)
            self.get_logger().info(f"Sent IMU → Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f} | Response: {response.status}")
        except grpc.RpcError as e:
            self.get_logger().error(f"gRPC failed: {e.details()}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()
