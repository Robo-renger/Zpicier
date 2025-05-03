package main

import (
	"context"
	"fmt"
	"log"
	"net"
	"os"

	pwm "zpicier/pb"       // from proto generation
	"zpicier/services/pca" // your PCA driver

	"google.golang.org/grpc"
)

// server implements the gRPC service defined in pwm.proto
type server struct {
	pwm.UnimplementedPWMServiceServer
	driver *pca.PCA
}

// SetPWM handles incoming RPC calls and sets the PWM
func (s *server) SetPWM(ctx context.Context, req *pwm.PWMRequest) (*pwm.PWMResponse, error) {
	err := s.driver.PWMWrite(int(req.ChannelId), float64(req.Speed))
	if err != nil {
		log.Printf("Failed to set PWM: %v\n", err)
		return &pwm.PWMResponse{Status: "ERROR: " + err.Error()}, nil
	}
	log.Printf("PWM set on channel %d to %dµs\n",int(req.ChannelId), req.Speed)
	return &pwm.PWMResponse{Status: "OK"}, nil
}

func main() {
	// Init periph.io
	// Set environment (optional)
	os.Setenv("ENV", "SIMULATION") // or REAL

	// Create PCA driver
	pwmDriver := pca.GetInstance(50.0)

	// Start gRPC server
	lis, err := net.Listen("tcp", ":50051")
	if err != nil {
		log.Fatalf("Failed to listen: %v", err)
	}

	grpcServer := grpc.NewServer()
	pwm.RegisterPWMServiceServer(grpcServer, &server{driver: pwmDriver})

	fmt.Println("gRPC server listening on :50051")
	if err := grpcServer.Serve(lis); err != nil {
		log.Fatalf("Failed to serve: %v", err)
	}
}
