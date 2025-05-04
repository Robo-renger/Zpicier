
import os

class Dispatcher:
    def __init__():
        pass
    
    @staticmethod
    def getIMU():
        if os.getenv("ENV") == "SIMULATION" or True:
            from simulation_services.SimIMU import SimIMU
            return SimIMU()
        else:
            from services.IMU import BNO085
            return BNO085()