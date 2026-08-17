import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from hal.factory import create_backend
from core.simulation import CoreSimulation
from safety.limits import JOINT_NAMES, TORQUE_LIMITS, POSITION_LIMITS

def test_motor_diagnostics():
    print("Running Motor Diagnostic Tests...")
    
    cfg = {
        "BACKEND": "sim",
        "model_path": "models/parth_humanoid/scene.xml",
        "CONTROL_RATE_HZ": 200,
        "timestep": 0.005
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    sim.start()
    
    state = backend.get_state()
    assert len(state.joint_positions) >= len(JOINT_NAMES), "State missing joints"
    
    # Check all 19 actuators are initialized within safe physical bounds
    for i, name in enumerate(JOINT_NAMES):
        pos = state.joint_positions[i]
        torque = state.joint_torques[i] if i < len(state.joint_torques) else 0.0
        p_min, p_max = POSITION_LIMITS[i]
        t_max = TORQUE_LIMITS[i]
        
        assert p_min - 0.1 <= pos <= p_max + 0.1, f"Joint {name} out of bounds: {pos}"
        assert abs(torque) <= t_max + 5.0, f"Joint {name} torque exceeded: {torque}"
        
    print(f"✅ All {len(JOINT_NAMES)} humanoid motors passed diagnostic bounds check.")
    sim.stop()

if __name__ == "__main__":
    test_motor_diagnostics()
