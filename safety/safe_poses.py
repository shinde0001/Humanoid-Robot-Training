# safety/safe_poses.py
import numpy as np
from .limits import NUM_JOINTS, JOINT_NAMES

# All zeros pose (stand straight)
ZERO_POSE = np.zeros(NUM_JOINTS)

# Stand pose (matching H1 geometry: hip_pitch = -0.4, knee = 0.8, ankle = -0.4)
STAND_POSE = np.zeros(NUM_JOINTS)
STAND_POSE[JOINT_NAMES.index("left_hip_pitch")] = -0.4
STAND_POSE[JOINT_NAMES.index("left_knee")] = 0.8
STAND_POSE[JOINT_NAMES.index("left_ankle")] = -0.4

STAND_POSE[JOINT_NAMES.index("right_hip_pitch")] = -0.4
STAND_POSE[JOINT_NAMES.index("right_knee")] = 0.8
STAND_POSE[JOINT_NAMES.index("right_ankle")] = -0.4

# Crouch / safe pose (deeper knee bend)
CROUCH_POSE = np.zeros(NUM_JOINTS)
CROUCH_POSE[JOINT_NAMES.index("left_hip_pitch")] = -0.7
CROUCH_POSE[JOINT_NAMES.index("left_knee")] = 1.4
CROUCH_POSE[JOINT_NAMES.index("left_ankle")] = -0.7

CROUCH_POSE[JOINT_NAMES.index("right_hip_pitch")] = -0.7
CROUCH_POSE[JOINT_NAMES.index("right_knee")] = 1.4
CROUCH_POSE[JOINT_NAMES.index("right_ankle")] = -0.7
