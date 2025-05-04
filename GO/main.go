package main

import (
	"fmt"
	"log"
	"net"
	"os"
	"sync"

	nodemanager "zpicier/core/node_manager"
	serverRegistery "zpicier/core/server_registery"
	"zpicier/scripts/switching_node"

	"google.golang.org/grpc"
)

func main() {
	os.Setenv("ENV", "SIMULATION")

	var wg sync.WaitGroup
	nodeManager := nodemanager.NewNodeManager(&wg) 

	// Register gRPC server
	wg.Add(1)
	go func() {
		defer wg.Done()

		lis, err := net.Listen("tcp", ":50051")
		if err != nil {
			log.Fatalf("Failed to listen: %v", err)
		}

		grpcServer := grpc.NewServer()
		serverRegistery.RegisterAll(grpcServer)

		fmt.Println("gRPC server running on :50051")
		if err := grpcServer.Serve(lis); err != nil {
			log.Fatalf("gRPC serve error: %v", err)
		}
	}()

	nodeManager.Register("switching", switching_node.Run)
	nodeManager.RunAll()

	wg.Wait()
}
