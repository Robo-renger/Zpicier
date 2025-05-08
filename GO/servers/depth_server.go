package servers

import (
	"context"

	dtos "zpicier/DTOs"
	depth_pb "zpicier/pb/depth"
)

type DepthServer struct {
	depth_pb.UnimplementedDepthServiceServer
	depthInstance *dtos.Depth
}

func NewDepthServer() *DepthServer {
	
	return &DepthServer{
		depthInstance : dtos.GetDepthInstance(),
	}
}

func (s *DepthServer) SetDepth(ctx context.Context, req *depth_pb.DepthRequest) (*depth_pb.DepthResponse, error) {
	s.depthInstance.Update(req.Depth, req.Pressure)
	// fmt.Printf("_______Received Depth: depth=%.2f, pressure=%.2f\n", req.Depth, req.Pressure)
	return &depth_pb.DepthResponse{Status: "OK"}, nil
}
