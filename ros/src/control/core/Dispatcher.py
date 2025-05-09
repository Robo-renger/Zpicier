
import os
from utils.Configurator import Configurator
from utils.EnvParams import EnvParams
class Dispatcher:
    def __init__():
        pass
    
    @staticmethod
    def getIMU():
        if EnvParams().ENV == "SIMULATION":
            from simulation_services.SimIMU import SimIMU
            return SimIMU()
        else:
            from services.IMU import BNO085
            return BNO085()
    
    @staticmethod
    def getDepth():
        if EnvParams().ENV == "SIMULATION":
            from simulation_services.SimDepth import SimDepth
            return SimDepth()
        else:
            from services.Depth import DepthSensor
            return DepthSensor()
    @staticmethod
    def getPWMDriver():
        if EnvParams().ENV == "SIMULATION":
            from simulation_services.SimPCA import SimPCA
            return SimPCA()
        else:
            from services.PCA import PCA
            return PCA.getInst()
    
    