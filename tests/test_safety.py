import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from safety.safety_controller import SafetyController
from safety.limits import TORQUE_LIMITS
from hal.interface import RobotState, IMUData

def test_safety_system():
    print("Running Safety System Tests...")
    
    # Initialize controller at 200Hz
    safety = SafetyController(dt=0.005)
    
    # 1. Test Torque Clamping
    extreme_torques = np.ones(19) * 1000.0 # 1000 Nm is way above limits
    current_pos = np.zeros(19)
    safe_torques = safety.check_and_clamp_torques(extreme_torques, current_pos)
    
    # Assert they are clamped
    assert np.all(safe_torques <= TORQUE_LIMITS), "Torques were not clamped to TORQUE_LIMITS"
    assert safe_torques[0] == 200, f"Expected 200, got {safe_torques[0]}"
    assert safe_torques[13] == 18, f"Expected 18, got {safe_torques[13]}"
    print("✅ Tier 1: Torque Clamping passed.")
    
    # 2. Test NaN Torque injection (should trigger E-STOP)
    nan_torques = np.ones(19)
    nan_torques[5] = np.nan
    safe_torques_nan = safety.check_and_clamp_torques(nan_torques, current_pos)
    assert safety.is_estopped(), "E-STOP not triggered on NaN torque"
    assert np.all(safe_torques_nan == 0), "Outputs not zeroed during E-STOP"
    print("✅ Tier 1: NaN Command E-STOP passed.")
    
    # Reset controller for next test
    safety.estop_active = False
    
    # 3. Test NaN State from Sensors
    bad_imu = IMUData(np.zeros(4), np.zeros(3), np.zeros(3))
    bad_state = RobotState(
        timestamp=1.0,
        joint_positions=np.zeros(19),
        joint_velocities=np.zeros(19),
        joint_torques=np.zeros(19),
        imu=bad_imu,
        contact_forces=np.zeros(2),
        battery_voltage=48.0
    )
    bad_state.joint_positions[2] = np.nan
    
    safety.check_state(bad_state)
    assert safety.is_estopped(), "E-STOP not triggered on NaN sensor reading"
    print("✅ Tier 2: Sensor NaN E-STOP passed.")
    
    print("All Safety System tests passed successfully!")

if __name__ == "__main__":
    test_safety_system()
