package thruster

import (
	"log"
	"strconv"
	"time"
	"zpicier/core/logger"
	pwmdriver "zpicier/services/pwm_driver"
	smoother "zpicier/services/smoother"
)

type Thruster struct {
	Angle int
	Channel int
	Min_value int
	Max_value int
	Current_value int
	pwmDriver *pwmdriver.PWMDriver
	smoother *smoother.Smoother
	logger *logger.Logger
}

func NewThruster(channel interface{}, min_value int, max_value int, angle int) *Thruster {
	var channelInt int

	switch v := channel.(type) {
	case int:
		channelInt = v
	case string:
		parsed, err := strconv.Atoi(v)
		if err != nil {
			log.Fatalf("Failed to convert channel from string to int: %v", err)
		}
		channelInt = parsed
	default:
		log.Fatalf("Invalid type for channel: %T", v)
	}
	return &Thruster{
		Channel: channelInt,
		Min_value: min_value,
		Max_value: max_value,
		Current_value: int((max_value+min_value)/2),
		smoother: smoother.NewSmoother(),
		Angle: angle,
		pwmDriver: pwmdriver.NewPWMDriver(),
		logger: logger.NewLogger(),
	}
}

func (t *Thruster) Init() {
	go func() {
		t.logger.LogInPlaceInfo("Thruster %d initialized", t.Channel)
		t.Output(1500)
		time.Sleep(3 * time.Second)
	}()
}

func (t *Thruster) ensureValue(value int) int {
	if value < t.Min_value {
		value = t.Min_value
	} else if value > t.Max_value {
		value = t.Max_value
	}
	return value
}

func (t * Thruster) Output(value int) {
	t.pwmDriver.PWMWrite(t.Channel, float64(t.ensureValue(value)))
}

func (t *Thruster) Drive(value int) {
	value = t.ensureValue(value)
	value = int(t.smoother.ExponentialSmooth(float64(value), float64(t.Current_value), 0.1, 0.1))
	t.Output(value)
	t.Current_value = value
}

func (t *Thruster) Stop() {
	t.Drive(1500)
}

