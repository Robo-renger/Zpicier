#!/usr/bin/env python3
import random
import time
from threading import Thread, Lock
from zope.interface import implementer
from interfaces.IDepthSensor import IDepthSensor

@implementer(IDepthSensor)
class SimDepth:
    def __init__(self, bus: int, fluid_density: int) -> None:
        self.bus = bus
        self._fluid_density = fluid_density
        self._pressure = 0.0
        self._depth = 0.0
        self._lock = Lock()
        self._running = True
        Thread(target=self._simulate, daemon=True).start()

    def _simulate(self):
        while self._running:
            with self._lock:
                self._pressure = random.uniform(900, 1100)  # hPa or mbar
                self._depth = random.uniform(0.0, 10.0)     # meters
            time.sleep(0.1)

    def setFluidDensity(self, density: int) -> None:
        with self._lock:
            self._fluid_density = density

    def getFluidDensity(self) -> int:
        with self._lock:
            return self._fluid_density

    def readData(self) -> None:
        """In simulation, data is updated automatically, so this does nothing."""
        pass

    def getPressure(self) -> float:
        with self._lock:
            return self._pressure

    def getDepth(self) -> float:
        with self._lock:
            return self._depth
