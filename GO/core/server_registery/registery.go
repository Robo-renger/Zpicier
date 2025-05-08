package core

import (
	depth_pb "zpicier/pb/depth"
	imu_pb "zpicier/pb/imu"
	joy_pb "zpicier/pb/joy"
	"zpicier/servers"

	"google.golang.org/grpc"
)

func RegisterAll(grpcServer *grpc.Server) {
	imu_pb.RegisterIMUServiceServer(grpcServer, servers.NewIMUServer())
	joy_pb.RegisterJoystickServiceServer(grpcServer, servers.NewJoyServer())
	depth_pb.RegisterDepthServiceServer(grpcServer, servers.NewDepthServer())
}
