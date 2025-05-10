package navigation_node

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"
	dtos "zpicier/DTOs"
	"zpicier/core/configurator"
	"zpicier/services/joystick"
	"zpicier/services/navigation"
	PID "zpicier/services/pid_controller"
	"zpicier/services/thruster"
)

type NavigationNode struct {
	thrusters map[string]*thruster.Thruster
	joystick  *joystick.Joystick
	ctx        context.Context
	cancel     context.CancelFunc
	wg         *sync.WaitGroup
	navigation *navigation.Navigation
	x float64
	y float64
	z float64
	roll float64
	pitch float64
	yaw float64
	pid_yaw *PID.PIDController
	pid_pitch *PID.PIDController
	pid_pitch_horizontal *PID.PIDController
	pid_pitch_vertical *PID.PIDController
	pid_heave *PID.PIDController
	pid_heave_live *PID.PIDController
	imu *dtos.IMU
	depth *dtos.Depth
	lastState string

}
var(
	x_out float64
	y_out float64
	heave_out float64
	roll_out float64
	pitch_out float64
	yaw_out float64
	activePID bool
	manualSetPoint bool
	fix_heading bool
	fix_tilting bool
	fix_heave bool
	is_rotating bool
	capture bool
	neutralPitch float64
)
func NewNode() *NavigationNode {
	ctx, cancel := context.WithCancel(context.Background())
	return &NavigationNode{
		thrusters: make(map[string]*thruster.Thruster),
		joystick:  joystick.GetInstance(),
		navigation: navigation.NewNavigation(),
		wg: new(sync.WaitGroup),
		ctx:        ctx,
		cancel:     cancel,
		pid_yaw: PID.NewPIDController(configurator.Get("yaw_KP"), configurator.Get("yaw_KI"), configurator.Get("yaw_KD")),
		pid_pitch: PID.NewPIDController(configurator.Get("pitch_KP"), configurator.Get("pitch_KI"), configurator.Get("pitch_KD")),
		pid_pitch_horizontal: PID.NewPIDController(configurator.Get("horizontal_pitch_KP"), configurator.Get("horizontal_pitch_KI"), configurator.Get("horizontal_pitch_KD")),
		pid_pitch_vertical: PID.NewPIDController(configurator.Get("vertical_pitch_KP"), configurator.Get("vertical_pitch_KI"), configurator.Get("vertical_pitch_KD")),
		pid_heave: PID.NewPIDController(configurator.Get("heave_KP"), configurator.Get("heave_KI"), configurator.Get("heave_KD")),
		pid_heave_live: PID.NewPIDController(configurator.Get("live_heave_KP"), configurator.Get("live_heave_KI"), configurator.Get("live_heave_KD")),
		imu: dtos.GetIMUInstance(),
		depth: dtos.GetDepthInstance(),
	}
}
func (n *NavigationNode) Init() error {
	activePID = true 
	manualSetPoint = false 
	fix_heading = false 
	fix_tilting = false 
	fix_heave = false 
	is_rotating = false 
	capture = false 
	n.thrusters = map[string]*thruster.Thruster{
		"front_right": thruster.NewThruster(configurator.Get("FRONT_RIGHT_PCA_CHANNEL"), 1100, 1900, 135),
		"front_left":  thruster.NewThruster(configurator.Get("FRONT_LEFT_PCA_CHANNEL"), 1100, 1900, 45),
		"back_right":  thruster.NewThruster(configurator.Get("BACK_RIGHT_PCA_CHANNEL"), 1100, 1900, 315),
		"back_left":   thruster.NewThruster(configurator.Get("BACK_LEFT_PCA_CHANNEL"), 1100, 1900, 225),
		"back":        thruster.NewThruster(configurator.Get("BACK_PCA_CHANNEL"), 1100, 1900, 90),
		"front":       thruster.NewThruster(configurator.Get("FRONT_PCA_CHANNEL"), 1100, 1900, 90),
	}
	n.calibratePitchNeutral()
	fmt.Println("Neutral pitch calibrated to:", neutralPitch)
	n.navigation.SetThrusters(n.thrusters)
	n.pid_heave.MinOutput = -1 
	n.pid_heave.MaxOutput = 1

	n.pid_heave_live.MinOutput = -1
	n.pid_heave_live.MaxOutput = 1

	n.pid_yaw.MinOutput = -1
	n.pid_yaw.MaxOutput = 1

	n.pid_pitch.MinOutput = -1
	n.pid_pitch.MaxOutput = 1

	n.pid_pitch_horizontal.MinOutput = -1
	n.pid_pitch_horizontal.MaxOutput = 1
	
	n.pid_pitch_vertical.MinOutput = -1
	n.pid_pitch_vertical.MaxOutput = 1

	return nil
}
func (n *NavigationNode) handle() {
	n.wg.Add(2)
	go n.startJoystickReader()
	go n.startNavigationLoop()
	n.wg.Wait()
}

func (n *NavigationNode) Kill() {
	n.cancel()
	n.joystick.Close()
	for _, thruster := range n.thrusters {
		thruster.Stop()
	}
}

func (n *NavigationNode) startJoystickReader() {
	defer n.wg.Done()
	for {
		select {
		case <-n.ctx.Done():
			return
		default:
			n.x, n.y, n.roll, n.pitch, n.yaw = n.joystick.GetAxes()
			if n.joystick.IsClicked("HEAVE_UP") {
				n.z = math.Min(1, n.z+0.07)
			} else if n.joystick.IsClicked("HEAVE_DOWN") {
				n.z = math.Max(-1, n.z-0.07)
			} else {
				n.z = 0
			}
			time.Sleep(10 * time.Millisecond)
		}
	}
}
func (n *NavigationNode) startNavigationLoop() {
	defer n.wg.Done()
	for {
	select {
	case <-n.ctx.Done():
		return
	default:
		// Update PID setpoints
		if !n.joystick.IsRestYawAxis() {
			n.pid_yaw.Setpoint = n.imu.Yaw
		}

		if !n.joystick.IsRestPitchAxis() {
			n.pid_pitch.Setpoint = neutralPitch
			n.pid_pitch_horizontal.Setpoint = neutralPitch
			n.pid_pitch_vertical.Setpoint = neutralPitch
		} else {
			n.pid_pitch.Setpoint = n.imu.Pitch
			n.pid_pitch_horizontal.Setpoint = n.imu.Pitch
			n.pid_pitch_vertical.Setpoint = n.imu.Pitch
		}

		if !n.joystick.IsRestHeaveAxis(n.z) {
			n.pid_heave.Setpoint = n.depth.Depth
			n.pid_heave_live.Setpoint = n.depth.Depth
		}

		// Stabilization logic
		var currentState string

		if activePID && n.joystick.IsRest(n.z) {
			currentState = "rest"
			if n.lastState != currentState {
				fmt.Println("Stabilizing at rest")
				n.lastState = currentState
			}
			n.stabalizeAtRest()
		} else if activePID && n.joystick.IsMovingHorizontally() && n.joystick.IsRestAxes(n.z) {
			currentState = "horizontal"
			if n.lastState != currentState {
				fmt.Println("Stabilizing at horizontal movement")
				n.lastState = currentState
			}
			n.stabalizeAtHorizontalMovement()
		} else if activePID && n.joystick.IsMovingVertically(n.z) && n.joystick.IsRestYawAxis() && n.joystick.IsRestPitchAxis() {
			currentState = "vertical"
			if n.lastState != currentState {
				fmt.Println("Stabilizing at vertical movement")
				n.lastState = currentState
			}
			n.stabalizeAtVerticalMovement()
		} else if activePID && !n.joystick.IsRestYawAxis() && n.joystick.IsRestPitchAxis() && n.joystick.IsRestHeaveAxis(n.z) {
			currentState = "heading"
			if n.lastState != currentState {
				fmt.Println("Stabilizing while heading")
				n.lastState = currentState
			}
			n.stabalizeWhileHeading()
		} else {
			currentState = "manual"
			if n.lastState != currentState {
				fmt.Println("Manual navigation")
				n.lastState = currentState
			}
			n.manualNavigation()
		}
		n.navigation.Navigate(x_out, y_out, pitch_out, n.roll, heave_out, yaw_out)
		time.Sleep(10 * time.Millisecond)
	}
}
}


func (n *NavigationNode) Run() {
	if err := n.Init(); err != nil {
		fmt.Printf("Failed to init navigation node: %v\n", err)
		return
	}

	go n.handle()
}
func (n *NavigationNode) Stop() {
	n.cancel()
}

func (n *NavigationNode) manualNavigation() {
	yaw_out = n.yaw
	heave_out = n.z
	pitch_out = n.pitch
	x_out = n.x
	y_out = n.y
}

func (n *NavigationNode) stabalizeAtRest() {
	yaw_out = n.pid_yaw.Stabilize(n.imu.Yaw)
	pitch_out = n.pid_pitch.Stabilize(n.imu.Pitch)
	heave_out = n.pid_heave.Stabilize(n.depth.Depth)

	x_out = n.x
	y_out = n.y
}
func (n *NavigationNode) stabalizeAtHorizontalMovement() {
	yaw_out = n.pid_yaw.Stabilize(n.imu.Yaw)
	pitch_out = n.pid_pitch.Stabilize(n.imu.Pitch)
	heave_out = n.pid_heave.Stabilize(n.depth.Depth)

	x_out = n.x
	y_out = n.y
}
func (n *NavigationNode) stabalizeAtVerticalMovement() {
	yaw_out = n.pid_yaw.Stabilize(n.imu.Yaw)
	pitch_out = n.pid_pitch.Stabilize(n.imu.Pitch)
	heave_out = n.z

	x_out = n.x
	y_out = n.y
}	
func (n *NavigationNode) stabalizeWhileHeading() {
	pitch_out = n.pid_pitch.Stabilize(n.imu.Pitch)
	heave_out = n.pid_heave.Stabilize(n.depth.Depth)

	x_out = n.x
	y_out = n.y
}

func (n *NavigationNode) calibratePitchNeutral() {
	readings := make([]float64, 0)
	maxCalibrationTime := 10 * time.Second
	start := time.Now()

	for time.Since(start) < maxCalibrationTime {
		if n.imu == nil {
			fmt.Println("IMU sensor not initialized, waiting...")
		} else {
			readings = append(readings, n.imu.Pitch)
		}
		time.Sleep(100 * time.Millisecond)
		if time.Since(start) >= 5*time.Second && len(readings) == 0 {
			// After 5s of trying, still no data
			neutralPitch = -6.7
			fmt.Println("Failed to get pitch data, using fallback.")
			return
		}
	}

	if len(readings) == 0 {
		neutralPitch = -6.7
		fmt.Println("No valid readings, using fallback.")
		return
	}

	neutralPitch = 0.0
	for _, reading := range readings {
		neutralPitch += reading
	}
	neutralPitch /= float64(len(readings))
	fmt.Println("Calibrated readings:", readings)
	fmt.Println("Final neutral pitch value:", neutralPitch)
}
