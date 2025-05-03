package pca

import (
	"fmt"
	"log"
	"os"
	"sync"
	"time"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
)

const (
	mode1Reg    = 0x00
	prescaleReg = 0xFE
	led0OnL     = 0x06
	address = 0x40
)

type PCA struct {
	device         *i2c.Dev
	simulationMode bool
	frequency      float64
}

var (
	instance *PCA
	once     sync.Once
)

// GetInstance returns a singleton PCA instance
func GetInstance(frequency float64) *PCA {
	once.Do(func() {
		env := os.Getenv("ENV")
		simMode := env == "SIMULATION"
		p := &PCA{
			simulationMode: simMode,
			frequency:      frequency,
		}

		if !simMode {
			if _, err := host.Init(); err != nil {
				log.Fatal(err)
			}

			bus, err := i2creg.Open("")
			if err != nil {
				log.Fatal(err)
			}
			p.device = &i2c.Dev{Addr: address, Bus: bus}

			if err := p.initialize(frequency); err != nil {
				log.Fatalf("Failed to initialize PCA9685: %v", err)
			}
		} else {
			fmt.Println("Running in simulation mode. PCA9685 not initialized.")
		}
		instance = p
	})
	return instance
}


func (p *PCA) initialize(freq float64) error {
	// Reset
	if err := p.writeReg(mode1Reg, 0x00); err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	// Sleep to set prescale
	if err := p.writeReg(mode1Reg, 0x10); err != nil {
		return err
	}

	prescale := byte(25000000/(4096*freq) - 1)
	if err := p.writeReg(prescaleReg, prescale); err != nil {
		return err
	}

	// Wake up and set auto-increment
	if err := p.writeReg(mode1Reg, 0xA1); err != nil {
		return err
	}

	return nil
}

func (p *PCA) writeReg(reg byte, val byte) error {
	return p.device.Tx([]byte{reg, val}, nil)
}

func (p *PCA) PWMWrite(channel int, microseconds float64) error {
	if channel < 0 || channel > 15 {
		return fmt.Errorf("channel must be between 0 and 15")
	}
	if p.simulationMode {
		fmt.Printf("[Simulation Mode] Writing %.2f us to channel %d\n", microseconds, channel)
		return nil
	}
	duty := p.microsecondsToDuty(microseconds)
	return p.setPWM(channel, 0, duty)
}

func (p *PCA) microsecondsToDuty(microseconds float64) uint16 {
	period := 1_000_000.0 / p.frequency
	return uint16((microseconds / period) * 4096)
}

func (p *PCA) setPWM(channel int, on, off uint16) error {
	base := byte(led0OnL + 4*channel)
	data := []byte{
		base,
		byte(on), byte(on >> 8),
		byte(off), byte(off >> 8),
	}
	return p.device.Tx(data, nil)
}

func (p *PCA) StopAll() {
	if p.simulationMode {
		fmt.Println("[Simulation Mode] Stopping all channels.")
		return
	}
	for ch := 0; ch < 16; ch++ {
		p.setPWM(ch, 0, 0)
	}
}

func (p *PCA) Close() {
	// No explicit deinit in Go + periph.io, but good to nullify
	p.device = nil
}
