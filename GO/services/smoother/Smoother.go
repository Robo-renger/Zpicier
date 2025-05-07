package smoother


type Smoother struct {

}

// NewSmoother creates a new Smoother instance
func NewSmoother() *Smoother {
	return &Smoother{}
}

func (s *Smoother) ExponentialSmooth(target float64, current float64, alpha float64, tolerance float64) float64 {
	if alpha < 0 || alpha > 1 {
		panic("Alpha must be between 0 and 1")
	}
	if target < current-tolerance || target > current+tolerance {
		return target
	}
	return alpha*target + (1-alpha)*current
}