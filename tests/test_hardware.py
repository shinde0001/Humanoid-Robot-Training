import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import pytest
from hal.factory import create_backend
from core.simulation import CoreSimulation
from hal.real_backend import HardwareNotConnectedError

def test_real_hardware_init_fails():
    print("Running Hardware Init Test...")
    
    cfg = {
        "BACKEND": "real",
        "CONTROL_RATE_HZ": 200,
        "timestep": 0.005
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    
    # Starting the simulation with real hardware unplugged must throw HardwareNotConnectedError
    try:
        sim.start()
        assert False, "Should have thrown HardwareNotConnectedError!"
    except HardwareNotConnectedError as e:
        print(f"✅ Successfully caught expected hardware disconnect: {e}")
        
if __name__ == "__main__":
    test_real_hardware_init_fails()
