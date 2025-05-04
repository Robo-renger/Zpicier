package servers

import (
	"context"
	"log"
	"sync"

	joy_pb "zpicier/pb/joy"
	joy "zpicier/services/joystick"
)

type JoyServer struct {
	joy_pb.UnimplementedJoystickServiceServer

	mu      sync.RWMutex
	buttons map[string]bool
	axes    struct {
		X, Y, Z         float64
		Roll, Pitch, Yaw float64
	}
	joystick *joy.Joystick
}

func NewJoyServer() *JoyServer {
	return &JoyServer{
		buttons: make(map[string]bool),
		joystick: joy.GetInstance(),
	}
}

// UpdateState receives joystick data from ROS2
func (s *JoyServer) UpdateState(ctx context.Context, req *joy_pb.JoystickRequest) (*joy_pb.JoystickResponse, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	// Save button map
	s.buttons = make(map[string]bool)
	for k, v := range req.Buttons {
		s.buttons[k] = v
	}

	// Save axes into the server's struct
	s.axes.X = req.AxisX
	s.axes.Y = req.AxisY
	s.axes.Z = req.AxisZ
	s.axes.Roll = req.Roll
	s.axes.Pitch = req.Pitch
	s.axes.Yaw = req.Yaw

	// Update the joystick state
	s.joystick.UpdateButtons(s.buttons)
	s.joystick.UpdateAxes(s.axes.X, s.axes.Y, s.axes.Z, s.axes.Roll, s.axes.Pitch, s.axes.Yaw)

	log.Printf("[Joystick] Buttons: %+v | Axes: X=%.2f Y=%.2f Z=%.2f Roll=%.2f Pitch=%.2f Yaw=%.2f",
		s.buttons, s.axes.X, s.axes.Y, s.axes.Z, s.axes.Roll, s.axes.Pitch, s.axes.Yaw)

	return &joy_pb.JoystickResponse{Status: "OK"}, nil
}

