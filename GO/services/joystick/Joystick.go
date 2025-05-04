package joystick

import "sync"

type Joystick struct {
	mu sync.RWMutex

	axisX, axisY, axisZ          float64
	axisPitch, axisYaw, axisRoll float64
	activeButtons                map[string]bool
}

var (
	instance *Joystick
	once     sync.Once
)

// GetInstance returns the global singleton Joystick
func GetInstance() *Joystick {
	once.Do(func() {
		instance = &Joystick{
			activeButtons: make(map[string]bool),
		}
	})
	return instance
}

// IsClicked returns the current state of a mechanism (button)
func (j *Joystick) IsClicked(mechanism string) bool {
	j.mu.RLock()
	defer j.mu.RUnlock()

	if clicked, exists := j.activeButtons[mechanism]; exists {
		return clicked
	}
	return false
}

// UpdateButtons replaces the internal button map
func (j *Joystick) UpdateButtons(buttons map[string]bool) {
	j.mu.Lock()
	defer j.mu.Unlock()

	j.activeButtons = make(map[string]bool)
	for k, v := range buttons {
		j.activeButtons[k] = v
	}
}

// UpdateAxes sets all axis values at once
func (j *Joystick) UpdateAxes(x, y, z, pitch, yaw, roll float64) {
	j.mu.Lock()
	defer j.mu.Unlock()

	j.axisX = x
	j.axisY = y
	j.axisZ = z
	j.axisPitch = pitch
	j.axisYaw = yaw
	j.axisRoll = roll
}

// GetAxes returns current axis values
func (j *Joystick) GetAxes() (x, y, z, pitch, yaw, roll float64) {
	j.mu.RLock()
	defer j.mu.RUnlock()

	return j.axisX, j.axisY, j.axisZ, j.axisPitch, j.axisYaw, j.axisRoll
}
