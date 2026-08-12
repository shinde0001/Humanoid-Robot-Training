import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from hal.factory import create_backend
from core.simulation import CoreSimulation

def test_data_recorder():
    print("Running Data Recorder Tests...")
    
    cfg = {
        "BACKEND": "sim",
        "model_path": "models/parth_humanoid/scene.xml",
        "CONTROL_RATE_HZ": 200,
        "timestep": 0.005
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    
    sim.start()
    
    # Step 100 times normally
    sim.step_loop(num_steps=100)
    
    # Check if file was saved
    assert os.path.exists("data/recordings/flight_data_normal.npz"), "Normal flight data not saved!"
    
    # Test Crash Data Dump
    sim.running = True # re-enable
    # Force E-STOP
    sim.force_manual_command("estop")
    
    # Run loop again, it should immediately E-STOP and dump
    sim.step_loop(num_steps=10)
    
    assert os.path.exists("data/recordings/crash_report.npz"), "Crash report not saved!"
    
    # Verify the contents of the crash report
    data = np.load("data/recordings/crash_report.npz")
    assert "timestamps" in data
    assert "positions" in data
    assert "torques" in data
    assert data["positions"].shape[1] == 19 # 19 joints
    
    print("✅ All Data Recorder tests passed successfully!")

if __name__ == "__main__":
    test_data_recorder()
