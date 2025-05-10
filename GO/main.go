package main

import (
	"fmt"
	"log"
	"net"
	"os"
	"os/signal"
	"sync"
	"syscall"

	"zpicier/core/configurator"
	EnvParams "zpicier/core/env_params"
	nodemanager "zpicier/core/node_manager"
	serverRegistery "zpicier/core/server_registery"

	"google.golang.org/grpc"
)

func main() {
	var wg sync.WaitGroup
	nodeManager := nodemanager.NewNodeManager(&wg)
	
	//Handling sigterm
	sigs := make(chan os.Signal, 1)
	signal.Notify(sigs, syscall.SIGINT, syscall.SIGTERM)

	// Goroutine to handle signal
	go func() {
		sig := <-sigs
		fmt.Printf("\nReceived signal: %s, exiting...\n", sig)
		nodeManager.KillAll()
		fmt.Println("Waiting for all nodes to finish...")
		fmt.Println("All nodes killed")
		os.Exit(0)
	}()
	// End of signal handling

	// Initialize configurator and environment parameters
	configurator.AddConfigPath("config/joystick_buttons.yaml")
	configurator.AddConfigPath("config/hardware_pins.yaml")
	configurator.AddConfigPath("config/pid_ks.yaml")
	configurator.AddConfigPath("config/changeable_modules.yaml")
	EnvParams.Init()
	EnvParams.Get("ENV")
	if err := configurator.Init(); err != nil {
		panic(err)
	}
	// End of initialization
	

	// Register gRPC server
	wg.Add(1)
	go func() {
		defer wg.Done()

		lis, err := net.Listen("tcp", ":50051")
		if err != nil {
			log.Fatalf("Failed to listen: %v", err)
		}

		grpcServer := grpc.NewServer()
		
		// Register all services
		serverRegistery.RegisterAll(grpcServer)
		// End of service registration

		fmt.Println("gRPC server running on :50051")
		if err := grpcServer.Serve(lis); err != nil {
			log.Fatalf("gRPC serve error: %v", err)
		}
	}()
	// End of gRPC server registration

	// Register and run all nodes
	nodeManager.RegisterAll()
	nodeManager.RunAll()
	// End of node registration and running


	wg.Wait()
}
