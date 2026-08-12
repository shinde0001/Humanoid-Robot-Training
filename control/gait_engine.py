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
        self.walk_speed_yaw = 0.0
        self.target_height = 0.98
        self.step_frequency = 1.5 # Hz
        self.step_height = 0.15 # rad amplitude
        
    def set_command(self, cmd_type: str, params: dict):
        self.current_state = cmd_type
        if cmd_type == 'walk':
            self.walk_speed_x = float(params.get('v_x', 0.0))
            self.walk_speed_y = float(params.get('v_y', 0.0))
            self.walk_speed_yaw = float(params.get('v_yaw', 0.0))
            self.target_height = 0.98
        elif cmd_type == 'crouch':
            self.walk_speed_x = 0.0
            self.walk_speed_y = 0.0
            self.walk_speed_yaw = 0.0
            self.target_height = 0.78
        elif cmd_type == 'stand':
            self.walk_speed_x = 0.0
            self.walk_speed_y = 0.0
            self.walk_speed_yaw = 0.0
            self.target_height = 0.98
            
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
            # Kinematic sinusoidal gait coordinated with STAND_POSE
            phase = 2.0 * np.pi * self.step_frequency * self.time_elapsed
            
            pos = STAND_POSE.copy()
            amp = self.step_height * (np.abs(self.walk_speed_x) + 0.5)
            
            # Left leg
            pos[JOINT_NAMES.index("left_hip_pitch")] += np.sin(phase) * amp
            pos[JOINT_NAMES.index("left_knee")] += np.abs(np.cos(phase)) * amp
            pos[JOINT_NAMES.index("left_ankle")] -= np.sin(phase) * amp * 0.5
            
            # Right leg
            pos[JOINT_NAMES.index("right_hip_pitch")] += np.sin(phase + np.pi) * amp
            pos[JOINT_NAMES.index("right_knee")] += np.abs(np.cos(phase + np.pi)) * amp
            pos[JOINT_NAMES.index("right_ankle")] -= np.sin(phase + np.pi) * amp * 0.5
            
            # Natural arm swings
            pos[JOINT_NAMES.index("left_shoulder_pitch")] -= np.sin(phase) * 0.3
            pos[JOINT_NAMES.index("right_shoulder_pitch")] -= np.sin(phase + np.pi) * 0.3
            
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        return self.target_positions, self.target_velocities
