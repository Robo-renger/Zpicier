package main

import (
	"fmt"
	"log"
	"net"
	"sync"

	"zpicier/core/configurator"
	EnvParams "zpicier/core/env_params"
	nodemanager "zpicier/core/node_manager"
	serverRegistery "zpicier/core/server_registery"

	"google.golang.org/grpc"
)

func main() {
	// Initialize configurator and environment parameters
	configurator.AddConfigPath("config/joystick_buttons.yaml")
	configurator.AddConfigPath("config/hardware_pins.yaml")
	EnvParams.Init()
	EnvParams.Get("ENV")
	if err := configurator.Init(); err != nil {
		panic(err)
	}
	// End of initialization
	
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

	nodeManager.RegisterAll()
	nodeManager.RunAll()

	wg.Wait()
}
