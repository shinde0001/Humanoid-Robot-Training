import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
from hal.factory import create_backend
from core.simulation import CoreSimulation

def test_control_and_commands():
    print("Running Control & Command Tests...")
    
    cfg = {
        "BACKEND": "sim",
        "model_path": "models/parth_humanoid/scene.xml",
        "CONTROL_RATE_HZ": 200,
        "timestep": 0.005
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    
    sim.start()
    
    # 1. Test Stand Command
    cmd_stand = json.dumps({"type": "stand"})
    sim.send_command_string(cmd_stand)
    
    print("Stepping 100 times in stand mode...")
    sim.step_loop(num_steps=100)
    
    # 2. Test Walk Command
    cmd_walk = json.dumps({"type": "walk", "v_x": 0.5})
    sim.send_command_string(cmd_walk)
    
    print("Stepping 200 times in walk mode...")
    sim.step_loop(num_steps=200)
    
    # 3. Test Crouch Command
    cmd_crouch = json.dumps({"type": "crouch"})
    sim.send_command_string(cmd_crouch)
    
    print("Stepping 100 times in crouch mode...")
    sim.step_loop(num_steps=100)
    
    # Check final state
    state = backend.get_state()
    print(f"Final Sim Time: {state.timestamp:.3f}s")
    print(f"Final Torques (mean abs): {abs(state.joint_torques).mean():.3f} Nm")
    
    assert not sim.safety.is_estopped(), "Simulation E-Stopped unexpectedly!"
    print("✅ All Control & Command tests passed successfully!")

if __name__ == "__main__":
    test_control_and_commands()
