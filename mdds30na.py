#!/usr/bin/env python3
"""
MDDS30 (Locked Anti-Phase) driver for Raspberry Pi - No Acceleration Version
———————————————————————————————————————————————
- two GPIOs (default 18 / 19)  →  AN1 / AN2
- value range −1.0 … +1.0      →  full-rev … full-fwd
- No acceleration - immediate on/off control
Requires: sudo pigpiod
"""

import pigpio

class MDDS30AntiPhase:
    def __init__(self, left_pin=18, right_pin=19, freq=20_000):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running (sudo pigpiod).")
        self.left_pin  = left_pin
        self.right_pin = right_pin
        self.freq      = freq
        self._span     = 1_000_000         # pigpio duty range
        
        self.stop()                        # idle at 50%

    def _apply(self, pin, val):
        val = max(-1.0, min(1.0, val))     # clamp
        duty = int((val * 0.5 + 0.5) * self._span)
        self.pi.hardware_PWM(pin, self.freq, duty)

    # public API -----------------------------------------------------------
    def set(self, left, right):
        """left, right ∈ [−1.0 … +1.0] - applied immediately"""
        left = max(-1.0, min(1.0, left))
        right = -max(-1.0, min(1.0, right))
        self._apply(self.left_pin, left)
        self._apply(self.right_pin, right)

    def stop(self):
        """Stop both motors immediately"""
        self.set(0.0, 0.0)

    def close(self):
        """Clean up resources"""
        self.stop()
        self.pi.stop()

    # context-manager sugar -----------------------------------------------
    def __enter__(self):  
        return self
    
    def __exit__(self, *exc): 
        self.close()

# quick demo --------------------------------------------------------------
if __name__ == "__main__":
    import time
    
    with MDDS30AntiPhase() as drv:
        print("Demo with immediate on/off control...")
        
        print("Full forward - ON")
        drv.set(1.0, 1.0)     # full forward immediately
        time.sleep(2)
        
        print("Spin - ON")
        drv.set(-0.6, 0.6)    # spin immediately
        time.sleep(2)
        
        print("Stop - OFF")
        drv.stop()
        time.sleep(1)
        
        print("Half speed forward - ON")
        drv.set(0.5, 0.5)
        time.sleep(2)
        
        print("Stop - OFF")
        drv.stop()
        
        print("Demo complete!")
