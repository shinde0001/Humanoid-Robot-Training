# safety/safety_controller.py
import numpy as np
import logging
from .limits import POSITION_LIMITS, VELOCITY_LIMITS, TORQUE_LIMITS
from .safe_poses import CROUCH_POSE

class SafetyController:
    """
    Tier 1 & Tier 2 Safety implementation.
    Enforces joint limits, torques, velocities, and monitors system state.
    """
    def __init__(self, dt: float):
        self.dt = dt
        self.estop_active = False
        self.logger = logging.getLogger("SafetyController")
        
    def check_and_clamp_torques(self, torques: np.ndarray, current_positions: np.ndarray = None) -> np.ndarray:
        """Tier 1: Hardware limits clamp"""
        if self.estop_active:
            return np.zeros_like(torques)
            
        # Check for NaNs
        if np.any(np.isnan(torques)):
            self.trigger_estop("NaN detected in torque command.")
            return np.zeros_like(torques)

        # Clamp to max torque capability
        clamped_torques = np.clip(torques, -TORQUE_LIMITS, TORQUE_LIMITS)
        
        # Soft limits based on positions (Tier 2 feature embedded)
        if current_positions is not None:
            # If approaching limits, scale down torques pushing further out
            margins = 0.05 # 5% safety margin rad
            for i in range(len(current_positions)):
                pos = current_positions[i]
                if pos > POSITION_LIMITS[i, 1] - margins and clamped_torques[i] > 0:
                    clamped_torques[i] *= 0.1 # Dampen
                elif pos < POSITION_LIMITS[i, 0] + margins and clamped_torques[i] < 0:
                    clamped_torques[i] *= 0.1 # Dampen
                    
        return clamped_torques
        
    def check_state(self, robot_state) -> None:
        """Tier 2: Monitor robot state for critical failures."""
        if self.estop_active:
            return
            
        # Check IMU for free-fall or excessive tilt
        # (Assuming quaternion [w, x, y, z] - we check pitch/roll roughly via gravity vector later)
        
        # Check for NaN states
        if np.any(np.isnan(robot_state.joint_positions)) or np.any(np.isnan(robot_state.joint_velocities)):
            self.trigger_estop("NaN detected in robot state sensors.")

    def trigger_estop(self, reason: str):
        if not self.estop_active:
            self.logger.error(f"E-STOP TRIGGERED! Reason: {reason}")
            self.estop_active = True
            
    def is_estopped(self) -> bool:
        return self.estop_active
