package servers

import (
	"context"

	dtos "zpicier/DTOs"
	imu_pb "zpicier/pb/imu"
)

type IMUServer struct {
	imu_pb.UnimplementedIMUServiceServer
	imu *dtos.IMU
}

func NewIMUServer() *IMUServer {
	
	return &IMUServer{
		imu : dtos.GetIMUInstance(),
	}
}

func (s *IMUServer) SetEulerAngles(ctx context.Context, req *imu_pb.IMURequest) (*imu_pb.IMUResponse, error) {
	// log.Printf("Received IMU: pitch=%.2f, roll=%.2f, yaw=%.2f", req.Pitch, req.Roll, req.Yaw)
	s.imu.Update(req.Pitch, req.Roll, req.Yaw)
	return &imu_pb.IMUResponse{Status: "OK"}, nil
}
