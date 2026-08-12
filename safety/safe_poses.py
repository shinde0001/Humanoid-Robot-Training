# safety/safe_poses.py
import numpy as np
from .limits import NUM_JOINTS, JOINT_NAMES

# All zeros pose (stand straight)
ZERO_POSE = np.zeros(NUM_JOINTS)

# Stand pose (slight bend in knees for readiness, arms down)
STAND_POSE = np.zeros(NUM_JOINTS)
STAND_POSE[JOINT_NAMES.index("left_knee")] = 0.3
STAND_POSE[JOINT_NAMES.index("right_knee")] = 0.3
STAND_POSE[JOINT_NAMES.index("left_hip_pitch")] = -0.15
STAND_POSE[JOINT_NAMES.index("right_hip_pitch")] = -0.15
STAND_POSE[JOINT_NAMES.index("left_ankle")] = -0.15
STAND_POSE[JOINT_NAMES.index("right_ankle")] = -0.15

# Crouch / safe pose for when losing balance
CROUCH_POSE = np.zeros(NUM_JOINTS)
CROUCH_POSE[JOINT_NAMES.index("left_knee")] = 1.0
CROUCH_POSE[JOINT_NAMES.index("right_knee")] = 1.0
CROUCH_POSE[JOINT_NAMES.index("left_hip_pitch")] = -0.5
CROUCH_POSE[JOINT_NAMES.index("right_hip_pitch")] = -0.5
CROUCH_POSE[JOINT_NAMES.index("left_ankle")] = -0.5
CROUCH_POSE[JOINT_NAMES.index("right_ankle")] = -0.5
CROUCH_POSE[JOINT_NAMES.index("torso")] = 0.2
