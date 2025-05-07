package vectorizer

import (
	"fmt"
	"math"
	"zpicier/services/thruster"
)


type Vectorizer struct {
	thrusters map[string]*thruster.Thruster
}
var (
	yaw_only = true
)
func NewVectorizer() *Vectorizer {
	return &Vectorizer{
		thrusters: make(map[string]*thruster.Thruster),
	}
}
func (v *Vectorizer) SetThrusters(thrusters map[string]*thruster.Thruster) {
	v.thrusters = thrusters
}

func (v *Vectorizer) Vectorize(x, y, z, roll, pitch, yaw float64) map[string]float64 {
	forward_contrib := 0.0
	strafe_contrib := 0.0
	yaw_contrib := yaw
	thruster_speeds := make(map[string]float64)
	horizontal_thrusters := make([]string, 0)
	vertical_thrusters := make([]string, 0)
	min_vals := make(map[string]float64)
	max_vals := make(map[string]float64)
	for label, thruster := range v.thrusters {
		min_vals[label] = float64(thruster.Min_value)
		max_vals[label] = float64(thruster.Max_value)
		if thruster.Angle != 90{
			horizontal_thrusters = append(horizontal_thrusters, label)
		}else{
			vertical_thrusters = append(vertical_thrusters, label)
		}
		if thruster.Angle == 135 || thruster.Angle == 315 {
			forward_contrib = -y*math.Sin(float64(thruster.Angle)*math.Pi/180) 
			strafe_contrib  = -x*math.Cos(float64(thruster.Angle)*math.Pi/180)
		}else{
			forward_contrib = -y*math.Cos(float64(thruster.Angle)*math.Pi/180) 
			strafe_contrib  = x*math.Sin(float64(thruster.Angle)*math.Pi/180)
		}
		if !((yaw_only && math.Abs(x) <= 0.3 && math.Abs(y) <= 0.3) || !yaw_only) {
			yaw_contrib = 0
		}

		speed := forward_contrib + strafe_contrib + yaw_contrib
		thruster_speeds[label] = speed
	}
	thruster_speeds["front"] = -(z-pitch)
	thruster_speeds["back"] = -(z+pitch)
	horizontal_thruster_speeds := make(map[string]float64)
	vertical_thruster_speeds := make(map[string]float64)
	for _, label := range horizontal_thrusters {
		if speed, ok := thruster_speeds[label]; ok {
			horizontal_thruster_speeds[label] = speed
		}
	}
	for _, label := range vertical_thrusters {
		if speed, ok := thruster_speeds[label]; ok {
			vertical_thruster_speeds[label] = speed
		}
	}
	max_horizontal_input := max(math.Abs(x), math.Abs(y), math.Abs(yaw))
	max_horizontal_speed := math.Inf(-1)
	for _, val := range horizontal_thruster_speeds {
		if val > max_horizontal_speed {
			max_horizontal_speed = val
		}
	}
	if max_horizontal_speed > 0{
		horizontal_scaling_factor := max_horizontal_input / max_horizontal_speed
		for label, speed := range horizontal_thruster_speeds {
			horizontal_thruster_speeds[label] = speed * horizontal_scaling_factor
		}
	}


	max_vertical_input := max(math.Abs(z), math.Abs(pitch))
	max_vertical_speed := math.Inf(-1)
	for _, val := range horizontal_thruster_speeds {
		if val > max_horizontal_speed {
			max_horizontal_speed = val
		}
	}
	if max_vertical_speed > 0{
		vertical_scaling_factor := max_vertical_input / max_vertical_speed
		for label, speed := range vertical_thruster_speeds {
			vertical_thruster_speeds[label] = speed * vertical_scaling_factor
		}
	}
	for label, speed := range horizontal_thruster_speeds {
		thruster_speeds[label] = float64(axisToPWM(speed, int(min_vals[label]), int(max_vals[label])))
	}
	for label, speed := range vertical_thruster_speeds {
		thruster_speeds[label] = float64(axisToPWM(speed, int(min_vals[label]), int(max_vals[label])))
	}
	return thruster_speeds

}

func axisToPWM(value float64, min_value int, max_value int) int {
	if !(value <= 1 || value >= -1) {
		fmt.Println("Value must be within range -1 and 1")
	}
	neutral := float64((max_value + min_value) / 2)
	if value < 0 {
		return int(neutral + (value * float64(neutral-float64(min_value))))
	}else{
		return int(neutral + (value * float64(float64(max_value)-float64(neutral))))
	}
}