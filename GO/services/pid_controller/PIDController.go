package pidcontroller

import (
	"log"
	"math"
	"strconv"
	"time"
)

type PIDController struct {
	Kp, Ki, Kd float64
	Setpoint   float64

	MinOutput float64
	MaxOutput float64

	IntegralMin float64
	IntegralMax float64

	SampleTime float64 // minimum dt in seconds

	lastOutput  float64
	lastTime    float64
	lastError   float64
	lastInput   float64
	integral    float64
	IsHeading bool
}

func parseConstants(kp, ki, kd interface{}) (float64, float64, float64) {
	var kpFloat, kiFloat, kdFloat float64

	switch v := kp.(type) {
	case int:
		kpFloat = float64(v)
	case string:
		parsed, err := strconv.ParseFloat(v, 64)
		if err != nil {
			log.Fatalf("Failed to convert channel from string to float: %v", err)
		}
		kpFloat = parsed
	default:
		log.Fatalf("Invalid type for channel: %T", v)
	}

	switch v := ki.(type) {
	case int:
		kiFloat = float64(v)
	case string:
		parsed, err := strconv.ParseFloat(v, 64)
		if err != nil {
			log.Fatalf("Failed to convert channel from string to float: %v", err)
		}
		kiFloat = parsed
	default:
		log.Fatalf("Invalid type for channel: %T", v)
	}

	switch v := kd.(type) {
	case int:
		kdFloat = float64(v)
	case string:
		parsed, err := strconv.ParseFloat(v, 64)
		if err != nil {
			log.Fatalf("Failed to convert channel from string to float: %v", err)
		}
		kdFloat = parsed
	default:
		log.Fatalf("Invalid type for channel: %T", v)
	}

	return kpFloat, kiFloat, kdFloat
}

func NewPIDController(kp, ki, kd interface{}) *PIDController {
	kpfloat, kifloat, kdfloat := parseConstants(kp, ki, kd)
	return &PIDController{
		Kp:          kpfloat,
		Ki:          kifloat,
		Kd:          kdfloat,
		MinOutput:   math.Inf(-1),
		MaxOutput:   math.Inf(1),
		IntegralMin: math.Inf(-1),
		IntegralMax: math.Inf(1),
		SampleTime:  0, // disabled
		IsHeading: false,
	}
}

func (pid *PIDController) Update(input float64) float64 {
	currentTime := float64(time.Now().UnixNano()) / 1e9
	dt := currentTime - pid.lastTime
	if pid.lastTime != 0 && pid.SampleTime > 0 && dt < pid.SampleTime {
		return pid.lastOutput
	}

	error := pid.Setpoint - input
	if pid.lastTime == 0 {
		pid.lastTime = currentTime
		pid.lastInput = input
		pid.lastError = error
		return 0
	}

	// Integral term
	pid.integral += error * dt
	if pid.integral > pid.IntegralMax {
		pid.integral = pid.IntegralMax
	} else if pid.integral < pid.IntegralMin {
		pid.integral = pid.IntegralMin
	}

	// Derivative on measurement (to avoid derivative kick)
	derivative := -(input - pid.lastInput) / dt

	output := pid.Kp*error + pid.Ki*pid.integral + pid.Kd*derivative

	if output > pid.MaxOutput {
		output = pid.MaxOutput
	} else if output < pid.MinOutput {
		output = pid.MinOutput
	}

	pid.lastOutput = output
	pid.lastTime = currentTime
	pid.lastInput = input
	pid.lastError = error

	return output
}

func (pid *PIDController) Reset() {
	pid.lastTime = 0
	pid.lastInput = 0
	pid.lastError = 0
	pid.integral = 0
	pid.lastOutput = 0
}

func (pid *PIDController) SetOutputLimits(min, max float64) {
	pid.MinOutput = min
	pid.MaxOutput = max
}

func (pid *PIDController) SetIntegralLimits(min, max float64) {
	pid.IntegralMin = min
	pid.IntegralMax = max
}

func (pid *PIDController) SetSampleTime(seconds float64) {
	pid.SampleTime = seconds
}


func angleDifference(a, b float64) float64 {
	diff := math.Mod(a-b+180, 360) - 180
	if diff < -180 {
		diff += 360
	}
	return diff
}

func (pid *PIDController) Stabilize(measured float64) float64 {
	if pid.IsHeading {
		// Calculate shortest angle diff for yaw wrap-around
		error := angleDifference(pid.Setpoint, measured)
		return pid.Update(measured+error)
	}
	return pid.Update(measured)
}