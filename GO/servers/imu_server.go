package servers

import (
	"context"
	"log"

	imu_pb "zpicier/pb/imu"
)

type IMUServer struct {
	imu_pb.UnimplementedIMUServiceServer
}

func NewIMUServer() *IMUServer {
	return &IMUServer{}
}

func (s *IMUServer) SetEulerAngles(ctx context.Context, req *imu_pb.IMURequest) (*imu_pb.IMUResponse, error) {
	log.Printf("Received IMU: pitch=%.2f, roll=%.2f, yaw=%.2f", req.Pitch, req.Roll, req.Yaw)
	return &imu_pb.IMUResponse{Status: "OK"}, nil
}
