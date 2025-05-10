package switching_node

import (
	"log"
	"sync"
	"time"

	"zpicier/core/configurator"
	"zpicier/core/logger"
	joy "zpicier/services/joystick"
	"zpicier/services/switching"
)

type SwitchingNode struct {
	switches map[string]*switching.Switch
	wg       sync.WaitGroup
	joystick *joy.Joystick
	logger   *logger.Logger
}

func NewNode() *SwitchingNode {
	return &SwitchingNode{
		switches: make(map[string]*switching.Switch),
		joystick: joy.GetInstance(),
		logger:   logger.NewLogger(),
	}
}

func (n *SwitchingNode) Init() error {
	// Example: initialize 3 switches for GPIO17, GPIO18, GPIO27
	pins := map[string]interface{}{
		"verticalGripper": configurator.Get("VERTICALGRIPPER_SWITCH"),
		"FRONTGRIPPER": configurator.Get("FRONTGRIPPER_SWITCH"),
		"leftGripper": configurator.Get("LEFTGRIPPER_SWITCH"),
	}

	for label, ref := range pins {
		sw, err := switching.NewSwitch(ref)
		if err != nil {
			return n.logger.LogInPlaceError("failed to init %s: %v", label, err)
		}
		n.switches[label] = sw
	}
	return nil
}

func (n *SwitchingNode) handle() {
	go n.runSwitches()
	n.wg.Wait()
}

func (n *SwitchingNode) Kill() {
	n.joystick.Close()
	for _, sw := range n.switches {
		sw.Close()
	}
	n.wg.Wait()
}

// To make this node runnable from nodeManager
func (n *SwitchingNode) Run() {
	node := NewNode()
	if err := node.Init(); err != nil {
		n.logger.LogInPlaceError("Failed to init switching node: %v\n", err)
		return
	}
	node.handle()
}

func (n *SwitchingNode) runSwitches() {
	defer n.wg.Done()
	for {
		n.frontGripper()
		n.verticalGripper()
		n.leftGripper()
		time.Sleep(100 * time.Millisecond) // polling delay
	}
}

func (n *SwitchingNode) frontGripper() {
	componentName := "FRONTGRIPPER"
	state := n.joystick.IsClicked(componentName)
	// fmt.Println("alo", state)
	if state {
		err := n.switches[componentName].Toggle()
		if err != nil {
			log.Printf("[%v] error: %v",componentName,err)
		}else {
			log.Printf("[%v] toggled --> ON? %v",componentName,n.switches[componentName].IsOn())
		}
	}
}
func (n *SwitchingNode) verticalGripper() {
	componentName := "VERTICALGRIPPER"
	state := n.joystick.IsClicked(componentName)
	if state {
		err := n.switches[componentName].Toggle()
		if err != nil {
			log.Printf("[%v] error: %v",componentName,err)
		}else {
			log.Printf("[%v] toggled --> ON? %v",componentName,n.switches[componentName].IsOn())
		}
	}
}
func (n *SwitchingNode) leftGripper() {
	componentName := "LEFTGRIPPER"
	state := n.joystick.IsClicked(componentName)
	if state {
		err := n.switches[componentName].Toggle()
		if err != nil {
			log.Printf("[%v] error: %v",componentName,err)
		}else {
			log.Printf("[%v] toggled --> ON? %v",componentName,n.switches[componentName].IsOn())
		}
	}
}

