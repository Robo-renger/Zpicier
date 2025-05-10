package nodemanager

import (
	"fmt"
	"sync"
	"zpicier/core/interfaces"
	"zpicier/scripts/navigation_node"
	"zpicier/scripts/switching_node"
)

type NodeManager struct {
	nodes map[string]interfaces.Node
	wg    *sync.WaitGroup
}

func NewNodeManager(wg *sync.WaitGroup) *NodeManager {
	return &NodeManager{
		nodes: make(map[string]interfaces.Node),
		wg:    wg,
	}
}

func (nm *NodeManager) Register(name string, node interfaces.Node) {
	nm.nodes[name] = node
}

func (nm *NodeManager) Run(name string) error {
	node, ok := nm.nodes[name]
	if !ok {
		return fmt.Errorf("node '%s' not registered", name)
	}
	nm.wg.Add(1)
	go func() {
		defer nm.wg.Done()
		node.Run()
	}()
	return nil
}

func (nm *NodeManager) RegisterAll() {
	nm.Register("switching", switching_node.NewNode())
	nm.Register("navigation", navigation_node.NewNode())

}
func (nm *NodeManager) Kill(name string) {
	node, ok := nm.nodes[name]
	if !ok {
		fmt.Printf("Node '%s' not registered\n", name)
		return
	}
	node.Kill()
	fmt.Printf("Node '%s' killed\n", name)
}
func (nm *NodeManager) KillAll() {
	for name := range nm.nodes {
		nm.Kill(name)
	}
}
func (nm *NodeManager) RunAll() {
	for name := range nm.nodes {
		fmt.Printf("Running node: %s\n", name)
		_ = nm.Run(name)
	}
}
