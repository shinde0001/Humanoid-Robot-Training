import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import json
from hal.factory import create_backend
from core.simulation import CoreSimulation

def test_validation_gate():
    print("Running Validation Gate Tests...")
    
    cfg = {
        "BACKEND": "sim",
        "model_path": "models/parth_humanoid/scene.xml",
        "CONTROL_RATE_HZ": 200,
        "timestep": 0.005
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    
    # Check default state
    assert sim.gait_engine.current_state == 'stand', "Default state should be stand"
    
    # 1. Propose command (should be gated)
    cmd_walk = json.dumps({"type": "walk", "v_x": 0.5})
    sim.propose_command_string(cmd_walk)
    assert sim.validation_gate.pending_command is not None, "Command was not queued"
    assert sim.gait_engine.current_state == 'stand', "State changed before approval!"
    print("✅ Validation Gate: Caught AI command.")
    
    # 2. Reject command
    sim.validation_gate.reject_command()
    assert sim.validation_gate.pending_command is None, "Command was not cleared"
    assert sim.gait_engine.current_state == 'stand', "State changed after rejection!"
    print("✅ Validation Gate: Handled rejection.")
    
    # 3. Approve command
    sim.propose_command_string(cmd_walk)
    sim.approve_pending_command()
    assert sim.gait_engine.current_state == 'walk', "State did not change after approval!"
    print("✅ Validation Gate: Handled approval.")
    
    # 4. Manual Override
    sim.force_manual_command("crouch")
    
    # We must step once to process the THINK loop
    sim.running = True
    # We'll just manually call the logic that happens in step_loop:
    override = sim.manual_override.get_override()
    if override:
        sim.gait_engine.set_command(override.type, {"v_x": override.v_x, "v_y": override.v_y, "v_yaw": override.v_yaw})
    
    assert sim.gait_engine.current_state == 'crouch', "Manual override did not apply immediately!"
    print("✅ Manual Override: Preempted autonomous state.")
    
    print("All Validation & Override tests passed successfully!")

if __name__ == "__main__":
    test_validation_gate()
