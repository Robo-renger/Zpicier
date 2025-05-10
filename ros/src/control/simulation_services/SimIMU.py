import random
import time
from zope.interface import implementer
from threading import Thread, Lock
from interfaces.IBNO085 import IBNO085
@implementer(IBNO085)
class SimIMU:
    def __init__(self):
        self._data = (0.0, 0.0, 0.0)
        self._lock = Lock()
        self._running = True
        Thread(target=self._simulate, daemon=True).start()

    def _simulate(self):
        while self._running:
            with self._lock:
                self._data = (
                    random.uniform(-180, 180),  # roll
                    random.uniform(-90, 90),    # pitch
                    random.uniform(-180, 180)   # yaw
                )
            time.sleep(0.1)

    def getEulerAngles(self):
        with self._lock:
            return self._data
    
    def enableFeature(self, feature: int) -> None:
        """Enables the specified feature.
        @param feature: The feature to enable.
        """
        print("Function not implemented in SIMULATION mode")
        
    def getAcceleration(self) -> tuple:
        """Returns the acceleration readings.
        @return The acceleration readings (x, y and z).
        """
        print("Function not implemented in SIMULATION mode")


    def getGyro(self) -> tuple:
        """Returns the gyroscope readings.
        @return The gyroscope readings (x, y and z).
        """
        print("Function not implemented in SIMULATION mode")

    def getMagnetometer(self) -> tuple:
        """Returns the magnetometer readings.
        @return The magnetometer readings (x, y and z).
        """
        print("Function not implemented in SIMULATION mode")


    def getQuaternion(self) -> tuple:
        """Returns the quaternion readings.
        @return The quaternion readings (x, y and z).
        """
        print("Function not implemented in SIMULATION mode")

    def Calibrate(self):
        """Calibrates the sensor.
        """
        print("Function not implemented in SIMULATION mode")

