package nodemanager

import (
	"fmt"
	"sync"
)

type NodeManager struct {
	nodes map[string]func()
	wg    *sync.WaitGroup
}

func NewNodeManager(wg *sync.WaitGroup) *NodeManager {
	return &NodeManager{
		nodes: make(map[string]func()),
		wg:    wg,
	}
}

func (nm *NodeManager) Register(name string, fn func()) {
	nm.nodes[name] = fn
}

func (nm *NodeManager) Run(name string) error {
	node, ok := nm.nodes[name]
	if !ok {
		return fmt.Errorf("node '%s' not registered", name)
	}
	nm.wg.Add(1)
	go func() {
		defer nm.wg.Done()
		node()
	}()
	return nil
}

func (nm *NodeManager) RunAll() {
	for name := range nm.nodes {
		fmt.Printf("Running node: %s\n", name)
		_ = nm.Run(name)
	}
}
