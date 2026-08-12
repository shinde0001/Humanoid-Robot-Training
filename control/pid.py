# control/pid.py
import numpy as np

class PDController:
    """
    Proportional-Derivative Controller for joint position tracking.
    """
    def __init__(self, kp: np.ndarray, kd: np.ndarray):
        self.kp = np.array(kp, dtype=np.float64)
        self.kd = np.array(kd, dtype=np.float64)
        self.num_joints = len(self.kp)
        
    def compute(self, target_positions: np.ndarray, target_velocities: np.ndarray,
                current_positions: np.ndarray, current_velocities: np.ndarray) -> np.ndarray:
        """
        Compute torque commands based on PD control law.
        tau = kp * (q_target - q_current) + kd * (v_target - v_current)
        """
        pos_error = target_positions - current_positions
        vel_error = target_velocities - current_velocities
        
        torques = (self.kp * pos_error) + (self.kd * vel_error)
        return torques
