# control/gait_engine.py
import numpy as np
import time
from safety.limits import NUM_JOINTS, JOINT_NAMES
from safety.safe_poses import STAND_POSE, CROUCH_POSE, ZERO_POSE

class GaitEngine:
    def __init__(self, dt: float):
        self.dt = dt
        self.time_elapsed = 0.0
        
        # State: 'stand', 'crouch', 'walk', 'zero'
        self.current_state = 'stand'
        
        # Target kinematic state
        self.target_positions = STAND_POSE.copy()
        self.target_velocities = np.zeros(NUM_JOINTS)
        
        # Walking parameters
        self.walk_speed_x = 0.0
        self.walk_speed_y = 0.0
        self.step_frequency = 1.5 # Hz
        self.step_height = 0.15 # rad amplitude
        
    def set_command(self, cmd_type: str, params: dict):
        self.current_state = cmd_type
        if cmd_type == 'walk':
            self.walk_speed_x = params.get('v_x', 0.0)
            self.walk_speed_y = params.get('v_y', 0.0)
            
    def update(self) -> tuple[np.ndarray, np.ndarray]:
        """Returns (target_positions, target_velocities)"""
        self.time_elapsed += self.dt
        
        if self.current_state == 'stand':
            self.target_positions = STAND_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        elif self.current_state == 'crouch':
            self.target_positions = CROUCH_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        elif self.current_state == 'zero':
            self.target_positions = ZERO_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        elif self.current_state == 'walk':
            # Simple kinematic sinusoidal gait on top of STAND_POSE
            phase = 2.0 * np.pi * self.step_frequency * self.time_elapsed
            
            # Left leg
            left_hip_pitch_idx = JOINT_NAMES.index("left_hip_pitch")
            left_knee_idx = JOINT_NAMES.index("left_knee")
            
            # Right leg
            right_hip_pitch_idx = JOINT_NAMES.index("right_hip_pitch")
            right_knee_idx = JOINT_NAMES.index("right_knee")
            
            # Start from stand pose base
            pos = STAND_POSE.copy()
            
            # Very naive sinusoidal trajectory for testing actuation in Phase 3
            amp = self.step_height * (np.abs(self.walk_speed_x) + 0.1)
            
            # Sine waves 180 degrees out of phase for walking
            pos[left_hip_pitch_idx] += np.sin(phase) * amp
            pos[left_knee_idx] += np.abs(np.cos(phase)) * amp  # Knee always bends same direction
            
            pos[right_hip_pitch_idx] += np.sin(phase + np.pi) * amp
            pos[right_knee_idx] += np.abs(np.cos(phase + np.pi)) * amp
            
            # Velocities could be derived mathematically, zeroing for naive PD control
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        return self.target_positions, self.target_velocities
