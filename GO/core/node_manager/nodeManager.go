package nodemanager

import (
	"sync"
	"zpicier/core/interfaces"
	"zpicier/core/logger"
	"zpicier/scripts/navigation_node"
	"zpicier/scripts/switching_node"
)

type NodeManager struct {
	nodes map[string]interfaces.Node
	wg    *sync.WaitGroup
	logger *logger.Logger
}

func NewNodeManager(wg *sync.WaitGroup) *NodeManager {
	return &NodeManager{
		nodes: make(map[string]interfaces.Node),
		wg:    wg,
		logger : logger.NewLogger(),
	}
}

func (nm *NodeManager) Register(name string, node interfaces.Node) {
	nm.nodes[name] = node
}

func (nm *NodeManager) Run(name string) error {
	node, ok := nm.nodes[name]
	if !ok {
		return nm.logger.LogInPlaceError("Node '%s' not registered",name)
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
		nm.logger.LogInPlaceWarning("Node '%s' not registered",name)
		return
	}
	node.Kill()
	nm.logger.LogInPlaceSuccess("Node '%s' killed",name)
}
func (nm *NodeManager) KillAll() {
	for name := range nm.nodes {
		nm.Kill(name)
	}
}
func (nm *NodeManager) RunAll() {
	for name := range nm.nodes {
		nm.logger.LogInPlaceSuccess("Running node '%s'",name)
		_ = nm.Run(name)
	}
}
