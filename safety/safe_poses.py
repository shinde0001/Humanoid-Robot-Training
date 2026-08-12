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

# Victory pose (arms raised high)
VICTORY_POSE = STAND_POSE.copy()
VICTORY_POSE[JOINT_NAMES.index("left_shoulder_pitch")] = -1.5
VICTORY_POSE[JOINT_NAMES.index("left_shoulder_roll")] = 0.8
VICTORY_POSE[JOINT_NAMES.index("left_elbow")] = 0.5
VICTORY_POSE[JOINT_NAMES.index("right_shoulder_pitch")] = -1.5
VICTORY_POSE[JOINT_NAMES.index("right_shoulder_roll")] = -0.8
VICTORY_POSE[JOINT_NAMES.index("right_elbow")] = 0.5

# Fight / Boxing guard stance
FIGHT_POSE = STAND_POSE.copy()
FIGHT_POSE[JOINT_NAMES.index("left_shoulder_pitch")] = -0.6
FIGHT_POSE[JOINT_NAMES.index("left_shoulder_roll")] = 0.2
FIGHT_POSE[JOINT_NAMES.index("left_elbow")] = 1.6
FIGHT_POSE[JOINT_NAMES.index("right_shoulder_pitch")] = -0.6
FIGHT_POSE[JOINT_NAMES.index("right_shoulder_roll")] = -0.2
FIGHT_POSE[JOINT_NAMES.index("right_elbow")] = 1.6

# Look Left pose (torso yaw rotated left)
LOOK_LEFT_POSE = STAND_POSE.copy()
LOOK_LEFT_POSE[JOINT_NAMES.index("torso")] = 1.0

# Look Right pose (torso yaw rotated right)
LOOK_RIGHT_POSE = STAND_POSE.copy()
LOOK_RIGHT_POSE[JOINT_NAMES.index("torso")] = -1.0

# Bow pose (respectful forward bow from waist)
BOW_POSE = STAND_POSE.copy()
BOW_POSE[JOINT_NAMES.index("left_hip_pitch")] = -0.8
BOW_POSE[JOINT_NAMES.index("right_hip_pitch")] = -0.8
BOW_POSE[JOINT_NAMES.index("left_knee")] = 0.5
BOW_POSE[JOINT_NAMES.index("right_knee")] = 0.5
