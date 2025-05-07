package joystick

import (
	"fmt"
	"strings"
	"sync"
	"zpicier/core/configurator"
)

type Joystick struct {
	mu sync.RWMutex

	axisX, axisY, axisZ          float64
	axisPitch, axisYaw, axisRoll float64
	activeButtons                map[string]bool
	buttonMappings 				 map[string]string 
}

var (
	instance *Joystick
	once     sync.Once
)

// GetInstance returns the global singleton Joystick
func GetInstance() *Joystick {
	once.Do(func() {
		mappings := make(map[string]string)
		for k, v := range configurator.GetAll() {
			if strings.HasPrefix(k, "BUTTON_") {
				k = strings.ToLower(k)
				fmt.Println("Adding button mapping:", k, "->", v)
				mappings[k] = v
			}
		}
			instance = &Joystick{
			activeButtons: make(map[string]bool),
			buttonMappings: mappings,
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
func (j *Joystick) UpdateButtons(raw map[string]bool) {
	j.mu.Lock()
	defer j.mu.Unlock()

	j.activeButtons = make(map[string]bool)
	for rawButton, pressed := range raw {
		if action, exists := j.buttonMappings[rawButton]; exists {
			j.activeButtons[action] = pressed
		}
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