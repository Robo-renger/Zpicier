package dtos

import "sync"

type PWM struct {
	mu     sync.RWMutex
	Values map[int]float64
}

var (
	PWMInstance *PWM
	oncePWM     sync.Once
)

func GetPWMInstance() *PWM {
	oncePWM.Do(func() {
		PWMInstance = &PWM{
			Values: make(map[int]float64),
		}
	})
	return PWMInstance
}

func (p *PWM) PWMWrite(channel int, microseconds float64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.Values[channel] = microseconds
}

func (p *PWM) GetAll() map[int]float64 {
	p.mu.RLock()
	defer p.mu.RUnlock()

	copied := make(map[int]float64)
	for k, v := range p.Values {
		copied[k] = v
	}
	return copied
}
