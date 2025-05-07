package dtos

import "sync"

type IMU struct {
	Pitch float64
	Roll  float64
	Yaw   float64
}

var (
	imuInstance *IMU
	once        sync.Once
)

// GetInstance returns the singleton IMU instance
func GetIMUInstance() *IMU {
	once.Do(func() {
		imuInstance = &IMU{}
	})
	return imuInstance
}

// Optional constructor if needed
func NewIMU(pitch, roll, yaw float64) *IMU {
	imu := GetIMUInstance()
	imu.Update(pitch, roll, yaw)
	return imu
}

func (imu *IMU) Update(pitch, roll, yaw float64) {
	imu.Pitch = pitch
	imu.Roll = roll
	imu.Yaw = yaw
}
