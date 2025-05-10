package navigation

import (
	"zpicier/services/thruster"
	"zpicier/services/vectorizer"
)

type Navigation struct {
	thrusters map[string]*thruster.Thruster
	Vectorizer *vectorizer.Vectorizer
}

func (n *Navigation) InitMotors() {
	for _, thruster := range n.thrusters {
		thruster.Init()
	}
}

func NewNavigation() *Navigation {
	return &Navigation{
		thrusters: make(map[string]*thruster.Thruster),
		Vectorizer: vectorizer.NewVectorizer(),
	}
}

func (n *Navigation) SetThrusters(thrusters map[string]*thruster.Thruster) {
	n.thrusters = thrusters
	n.Vectorizer.SetThrusters(thrusters)
}

func (n *Navigation) Navigate(x, y, z, roll, pitch, yaw float64) {
	// fmt.Println("Navigating with inputs:", x, y, z, roll, pitch, yaw)
	vectorized := n.Vectorizer.Vectorize(x, y, z, roll, pitch, yaw)
	if vectorized != nil {
		// fmt.Println("Vectorized values:", vectorized)
		for label, thruster := range n.thrusters {
			if value, exists := vectorized[label]; exists {
				thruster.Drive(int(value))
			}
		}
	}
}