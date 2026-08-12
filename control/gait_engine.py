# control/gait_engine.py
import numpy as np
import time
from safety.limits import NUM_JOINTS, JOINT_NAMES
from safety.safe_poses import (
    STAND_POSE, CROUCH_POSE, ZERO_POSE, VICTORY_POSE,
    FIGHT_POSE, LOOK_LEFT_POSE, LOOK_RIGHT_POSE, BOW_POSE
)

class GaitEngine:
    def __init__(self, dt: float):
        self.dt = dt
        self.time_elapsed = 0.0
        self.action_time = 0.0
        
        # State: 'stand', 'crouch', 'walk', 'jump', 'wave', 'punch', 'victory', 'fight', 'look_left', 'look_right', 'scan', 'bow', 'zero'
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
        
    def set_command(self, cmd_type: str, params: dict = None):
        if params is None:
            params = {}
        if self.current_state != cmd_type:
            self.action_time = 0.0
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
        elif cmd_type in ['wave', 'punch', 'victory', 'fight', 'look_left', 'look_right', 'scan', 'bow']:
            self.walk_speed_x = 0.0
            self.walk_speed_y = 0.0
            self.walk_speed_yaw = 0.0
            self.target_height = 0.98
            
    def update(self) -> tuple[np.ndarray, np.ndarray]:
        """Returns (target_positions, target_velocities)"""
        self.time_elapsed += self.dt
        self.action_time += self.dt
        
        if self.current_state == 'stand':
            self.target_positions = STAND_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'crouch':
            self.target_positions = CROUCH_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.78
            
        elif self.current_state == 'zero':
            self.target_positions = ZERO_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'jump':
            pos = STAND_POSE.copy()
            if self.action_time < 0.35:
                # Phase 1: Deep crouch prep & arm windup
                pos[JOINT_NAMES.index('left_hip_pitch')] = -0.7
                pos[JOINT_NAMES.index('left_knee')] = 1.3
                pos[JOINT_NAMES.index('left_ankle')] = -0.6
                pos[JOINT_NAMES.index('right_hip_pitch')] = -0.7
                pos[JOINT_NAMES.index('right_knee')] = 1.3
                pos[JOINT_NAMES.index('right_ankle')] = -0.6
                pos[JOINT_NAMES.index('left_shoulder_pitch')] = 0.4
                pos[JOINT_NAMES.index('right_shoulder_pitch')] = 0.4
                self.target_height = 0.70
            elif self.action_time < 0.70:
                # Phase 2: Explosive upward launch & arm thrust
                pos[JOINT_NAMES.index('left_hip_pitch')] = -0.1
                pos[JOINT_NAMES.index('left_knee')] = 0.2
                pos[JOINT_NAMES.index('left_ankle')] = -0.1
                pos[JOINT_NAMES.index('right_hip_pitch')] = -0.1
                pos[JOINT_NAMES.index('right_knee')] = 0.2
                pos[JOINT_NAMES.index('right_ankle')] = -0.1
                pos[JOINT_NAMES.index('left_shoulder_pitch')] = -1.2
                pos[JOINT_NAMES.index('right_shoulder_pitch')] = -1.2
                self.target_height = 1.35
            elif self.action_time < 1.20:
                # Phase 3: Airborne flight tuck
                pos[JOINT_NAMES.index('left_hip_pitch')] = -0.3
                pos[JOINT_NAMES.index('left_knee')] = 0.6
                pos[JOINT_NAMES.index('right_hip_pitch')] = -0.3
                pos[JOINT_NAMES.index('right_knee')] = 0.6
                self.target_height = 1.20
            elif self.action_time < 1.60:
                # Phase 4: Soft landing absorption
                pos[JOINT_NAMES.index('left_hip_pitch')] = -0.5
                pos[JOINT_NAMES.index('left_knee')] = 1.0
                pos[JOINT_NAMES.index('right_hip_pitch')] = -0.5
                pos[JOINT_NAMES.index('right_knee')] = 1.0
                self.target_height = 0.90
            else:
                # Phase 5: Settle back to stand
                pos = STAND_POSE.copy()
                self.target_height = 0.98
                self.current_state = 'stand'
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            
        elif self.current_state == 'wave':
            # Wave right hand/arm to greet
            pos = STAND_POSE.copy()
            pos[JOINT_NAMES.index("right_shoulder_pitch")] = -0.6
            pos[JOINT_NAMES.index("right_shoulder_roll")] = -1.2
            pos[JOINT_NAMES.index("right_elbow")] = 1.4
            pos[JOINT_NAMES.index("right_shoulder_yaw")] = np.sin(self.time_elapsed * 6.0) * 0.45
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'punch':
            # Boxing combo with alternating left/right punches and torso twist
            phase = 2.0 * np.pi * 2.0 * self.time_elapsed
            sin_p = np.sin(phase)
            pos = STAND_POSE.copy()
            pos[JOINT_NAMES.index("torso")] = sin_p * 0.35
            if sin_p > 0:
                pos[JOINT_NAMES.index("left_shoulder_pitch")] = -1.5 * sin_p
                pos[JOINT_NAMES.index("left_elbow")] = 0.2
                pos[JOINT_NAMES.index("right_shoulder_pitch")] = -0.4
                pos[JOINT_NAMES.index("right_elbow")] = 1.8
            else:
                pos[JOINT_NAMES.index("right_shoulder_pitch")] = 1.5 * sin_p
                pos[JOINT_NAMES.index("right_elbow")] = 0.2
                pos[JOINT_NAMES.index("left_shoulder_pitch")] = -0.4
                pos[JOINT_NAMES.index("left_elbow")] = 1.8
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'victory':
            pos = VICTORY_POSE.copy()
            # Subtle victory sway
            pos[JOINT_NAMES.index("torso")] = np.sin(self.time_elapsed * 2.0) * 0.1
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'fight':
            self.target_positions = FIGHT_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'look_left':
            self.target_positions = LOOK_LEFT_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'look_right':
            self.target_positions = LOOK_RIGHT_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'scan':
            # Smooth continuous radar head & torso sweep
            pos = STAND_POSE.copy()
            pos[JOINT_NAMES.index("torso")] = np.sin(self.time_elapsed * 1.5) * 1.0
            self.target_positions = pos
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.98
            
        elif self.current_state == 'bow':
            self.target_positions = BOW_POSE.copy()
            self.target_velocities = np.zeros(NUM_JOINTS)
            self.target_height = 0.95
            
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
            self.target_height = 0.98
            
        return self.target_positions, self.target_velocities
