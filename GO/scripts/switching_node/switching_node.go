package switching_node

import (
	"fmt"
	"log"
	"sync"
	"time"

	"zpicier/core/interfaces"
	joy "zpicier/services/joystick"
	"zpicier/services/switching"
)

type SwitchingNode struct {
	switches map[string]*switching.Switch
	wg       sync.WaitGroup
	joystick *joy.Joystick
}

func NewNode() *SwitchingNode {
	return &SwitchingNode{
		switches: make(map[string]*switching.Switch),
		joystick: joy.GetInstance(),
	}
}

func (n *SwitchingNode) Init() error {
	// Example: initialize 3 switches for GPIO17, GPIO18, GPIO27
	pins := map[string]interface{}{
		"main_light": 17,
		"pump":       18,
		"valve":      "GPIO27",
	}

	for label, ref := range pins {
		sw, err := switching.NewSwitch(ref)
		if err != nil {
			return fmt.Errorf("failed to init %s: %v", label, err)
		}
		n.switches[label] = sw
	}
	return nil
}

func (n *SwitchingNode) Run() {
	n.wg.Add(1)
	go n.runSwitches()
	n.wg.Wait()
}


func (n *SwitchingNode) runSwitches() {
	defer n.wg.Done()
	for {
		n.mainLight()
		n.pump()
		n.valve()
		time.Sleep(100 * time.Millisecond) // polling delay
	}
}

func (n *SwitchingNode) mainLight() {
	if n.switches["main_light"] == nil {
		log.Printf("[Main Light] ERROR: switch not initialized")
		return
	}
	state := n.joystick.IsClicked("main_light")
	if state {
		err := n.switches["main_light"].Toggle()
		if err != nil {
			log.Printf("[Main Light] error: %v", err)
		}else {
			log.Printf("[Main Light] toggled --> ON? %v", n.switches["main_light"].IsOn())
		}
	}
}
func (n *SwitchingNode) pump() {
	if n.switches["pump"] == nil {
		log.Printf("[Pump] ERROR: switch not initialized")
		return
	}
	state := n.joystick.IsClicked("pump")
	if state {
		err := n.switches["pump"].Toggle()
		if err != nil {
			log.Printf("[Pump] error: %v", err)
		}else {
			log.Printf("[Pump] toggled --> ON? %v", n.switches["pump"].IsOn())
		}
	}
}
func (n *SwitchingNode) valve() {
	if n.switches["valve"] == nil {
		log.Printf("[Valve] ERROR: switch not initialized")
		return
	}
	state := n.joystick.IsClicked("valve")
	if state {
		err := n.switches["valve"].Toggle()
		if err != nil {
			log.Printf("[Valve] error: %v", err)
		}else {
			log.Printf("[Valve] toggled --> ON? %v", n.switches["valve"].IsOn())
		}
	}
}

var _ interfaces.Node = (*SwitchingNode)(nil)

func Run() {
	node := NewNode()
	if err := node.Init(); err != nil {
		fmt.Printf("Failed to init switching node: %v\n", err)
		return
	}
	node.Run()
}