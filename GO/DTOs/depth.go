package dtos

import "sync"

type Depth struct {
	Depth float64
	Pressure  float64
}

var (
	depthInstance *Depth
	onceDepth        sync.Once
)

// GetInstance returns the singleton IMU instance
func GetDepthInstance() *Depth {
	onceDepth.Do(func() {
		depthInstance = &Depth{}
	})
	return depthInstance
}

// Optional constructor if needed
func NewDepth(depth,pressure float64) *Depth {
	depthinst := GetDepthInstance()
	depthinst.Update(depth,pressure)
	return depthinst
}

func (depth *Depth) Update(depthval, pressure float64) {
	depth.Depth = depthval
	depth.Pressure = pressure
}
