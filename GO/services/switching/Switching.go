package switching

import (
	"fmt"
	"os"
	"strconv"
	"sync"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/host/v3"
)

type Switch struct {
	pinName     string
	pin         gpio.PinIO // nil in simulation
	simulated   bool
	mu          sync.Mutex
	state       bool
}

// NewSwitch accepts GPIO pin name (e.g., "GPIO18") or int (e.g., 18)
func NewSwitch(pinRef interface{}) (*Switch, error) {
	var pinName string
	switch v := pinRef.(type) {
	case string:
		pinName = v
	case int:
		pinName = "GPIO" + strconv.Itoa(v)
	default:
		return nil, fmt.Errorf("unsupported pin type %T, must be string or int", pinRef)
	}

	// Check for simulation mode
	if os.Getenv("ENV") == "SIMULATION" {
		fmt.Printf("[SIMULATION] Initializing switch on %s\n", pinName)
		return &Switch{
			pinName:   pinName,
			simulated: true,
			state:     false,
		}, nil
	}

	// Init real GPIO
	if _, err := host.Init(); err != nil {
		return nil, fmt.Errorf("failed to init periph: %v", err)
	}

	pin := gpioreg.ByName(pinName)
	if pin == nil {
		return nil, fmt.Errorf("GPIO pin %s not found", pinName)
	}

	if err := pin.Out(gpio.Low); err != nil {
		return nil, fmt.Errorf("failed to set pin low: %v", err)
	}

	return &Switch{
		pinName: pinName,
		pin:     pin,
		state:   false,
	}, nil
}

func (s *Switch) On() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.simulated {
		if err := s.pin.Out(gpio.High); err != nil {
			return err
		}
	}
	s.state = true
	return nil
}

func (s *Switch) Off() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.simulated {
		if err := s.pin.Out(gpio.Low); err != nil {
			return err
		}
	}
	s.state = false
	return nil
}

func (s *Switch) Toggle() error {
	s.mu.Lock()
	defer s.mu.Unlock()

	var err error
	if s.state {
		if !s.simulated {
			err = s.pin.Out(gpio.Low)
		}
		s.state = false
	} else {
		if !s.simulated {
			err = s.pin.Out(gpio.High)
		}
		s.state = true
	}
	return err
}

func (s *Switch) IsOn() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.state
}

func (s *Switch) IsOff() bool {
	return !s.IsOn()
}
