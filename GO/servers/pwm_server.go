package servers

import (
	"context"

	dtos "zpicier/DTOs"
	pwm_pb "zpicier/pb/pwm"
)

type PWMServer struct {
	pwm_pb.UnimplementedPWMServiceServer
	pwm *dtos.PWM
}

func NewPWMServer() *PWMServer {
	return &PWMServer{
		pwm: dtos.GetPWMInstance(),
	}
}

func (s *PWMServer) GetPWM(ctx context.Context, req *pwm_pb.GetPWMRequest) (*pwm_pb.PWMValues, error) {
	values := make([]*pwm_pb.PWMEntry, 0)

	pwmMap := s.pwm.GetAll()

	for ch, val := range pwmMap {
		values = append(values, &pwm_pb.PWMEntry{
			Channel:      int32(ch),
			Microseconds: val,
		})
	}

	return &pwm_pb.PWMValues{Values: values}, nil
}
