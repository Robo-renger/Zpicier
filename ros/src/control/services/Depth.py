#!/usr/bin/env python3
from ms5837 import MS5837_02BA
from zope.interface import implementer
from interfaces.IDepthSensor import IDepthSensor
from exceptions.SensorInitializationError import SensorInitializationError
from exceptions.SensorReadError import SensorReadError
# from services.Logger import Logger
# from DTOs.LogSeverity import LogSeverity
from utils.Logger import Logger
@implementer(IDepthSensor)
class DepthSensor:
    def __init__(self, bus: int = 0, fluid_density: int = 1000) -> None:
        """
        Initializes the DepthSensor. 
        Args:
            bus (int): The bus number for the sensor.
            fluid_density (int): The density of the fluid (default is 1000 kg/m³ for freshwater).
        Raises:
            SensorInitializationError: If the sensor fails to initialize.
        """
        self.sensor = MS5837_02BA(bus) # Specify bus
        self.logger = Logger()
        if not self.sensor.init():
            self.logger.logErrorInPlace("DepthSensor -> Depth sensor initialization failed")
            # Logger.logToFile(LogSeverity.ERROR, "Depth sensor initialization failed.", "DepthSensor")
            # Logger.logToGUI(LogSeverity.ERROR, "Depth sensor initialization failed.", "DepthSensor")
            raise SensorInitializationError("Depth sensor initialization failed.")
        
        self._fluid_density = fluid_density

    def setFluidDensity(self, density: int) -> None:
        """Sets the fluid density incase of different environmets like salt water.
        @param density: density of the fluid the sensor will be soaked in."""
        self._fluid_density = density

    def getFluidDensity(self) -> int:
        """Get the already set fluid density
        @return fluid density"""
        return self._fluid_density

    def readData(self) -> None:
        """
        Reads data from the sensor.
        Raises: SensorReadError: If the sensor fails to read data.
        """
        try:
            self.sensor.read()
        except Exception as e:
            self.logger.logErrorInPlace(f"DepthSensor -> Failed to read sensor data: {str(e)}")
            # Logger.logToFile(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "DepthSensor")
            # Logger.logToGUI(LogSeverity.ERROR, f"Failed to read sensor data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read depth sensor data: {str(e)}")
    
    def getPressure(self) -> float:
        """
        Returns the pressure reading from the sensor.
        Raises: SensorReadError: If the sensor fails to read pressure data.
        """
        try:
            return self.sensor.pressure()
        except Exception as e:
            self.logger.logErrorInPlace(f"DepthSensor -> Failed to read pressure data: {str(e)}")
            # Logger.logToFile(LogSeverity.ERROR, f"Failed to read pressure data: {str(e)}", "DepthSensor")
            # Logger.logToGUI(LogSeverity.ERROR, f"Failed to read pressure data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read pressure data: {str(e)}")

    def getDepth(self) -> float:
        """
        Returns the depth reading from the sensor.
        Raises: SensorReadError: If the sensor fails to read depth data.
        """
        try: 
            return self.sensor.depth()
        except Exception as e:
            self.logger.logErrorInPlace(f"DepthSensor -> Failed to read depth data: {str(e)}")
            # Logger.logToFile(LogSeverity.ERROR, f"Failed to read depth data: {str(e)}", "DepthSensor")
            # Logger.logToGUI(LogSeverity.ERROR, f"Failed to read depth data: {str(e)}", "DepthSensor")
            raise SensorReadError(f"Failed to read depth data: {str(e)}")