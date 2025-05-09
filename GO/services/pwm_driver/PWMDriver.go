package pwmdriver

import (
	dtos "zpicier/DTOs"
	"zpicier/core/configurator"
	"zpicier/services/pca"
)

type PWMDriver struct {
	pwmDTO *dtos.PWM
	pca *pca.PCA
}

func NewPWMDriver() *PWMDriver {
	pwmDriverType := configurator.Get("PWM_DRIVER")
	if pwmDriverType == "PCA/GO" {
		return &PWMDriver{
			pwmDTO: nil,
			pca: pca.GetInstance(50),
		}
	}else if pwmDriverType == "PCA/ROS" {
		return &PWMDriver{
			pwmDTO: dtos.GetPWMInstance(),
			pca: nil,
		}
	}
	return &PWMDriver{
		pwmDTO: dtos.GetPWMInstance(),
		pca: pca.GetInstance(50),
	}
}

func (p *PWMDriver) PWMWrite(ch int, value float64) {
	if p.pwmDTO != nil {
		p.pwmDTO.PWMWrite(ch, value)
	} else if p.pca != nil {
		p.pca.PWMWrite(ch, value)
	}
}

func (p *PWMDriver) StopAll(){
	if p.pwmDTO != nil {
		for i := 0; i < len(p.pwmDTO.Values); i++ {
			p.pwmDTO.PWMWrite(i, 0)
		}
		} else if p.pca != nil {
		p.pca.StopAll()
		
	}
}