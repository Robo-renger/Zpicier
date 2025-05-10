#!/usr/bin/env python3
from zope.interface import implementer
from interfaces.PWMDriver import PWMDriver

@implementer(PWMDriver)
class SimPCA:
    def __init__(self):
        self.frequency = 50  # Typical for servos
        self.period_microseconds = 1_000_000 // self.frequency  # e.g., 20,000us for 50Hz
        self.channels = [0] * 16  # Simulated duty cycles for 16 channels
        print("SIMPCA initialized (simulated PCA9685)")

    def microsecondsToDutycycle(self, microseconds: int) -> int:
        """
        Converts microseconds to a 16-bit duty cycle (0-65535) based on 20ms period.
        """
        duty_cycle = int((microseconds / self.period_microseconds) * 65535)
        duty_cycle = min(max(duty_cycle, 0), 65535)  # Clamp to valid range
        return duty_cycle

    def PWMWrite(self, channel: int, microseconds: int) -> None:
        if not 0 <= channel <= 15:
            raise ValueError(f"Channel {channel} is out of range (0-15)")
        
        duty_cycle = self.microsecondsToDutycycle(microseconds)
        self.channels[channel] = duty_cycle
        # print(f"[SIM] Channel {channel} set to {microseconds}us (duty {duty_cycle})")

    def stopAll(self) -> None:
        for i in range(16):
            self.channels[i] = 0
        print("[SIM] All channels stopped (duty cycles set to 0)")

    def close(self) -> None:
        self.stopAll()
        print("[SIM] SIMPCA resources released (simulated)")
