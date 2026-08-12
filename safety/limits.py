# safety/limits.py
import numpy as np

# Total actuated joints
NUM_JOINTS = 19

# Ordered matching the MuJoCo actuators
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    "torso",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow"
]

# Position limits (radians) [min, max]
POSITION_LIMITS = np.array([
    [-0.43, 0.43],    # left_hip_yaw
    [-0.43, 0.43],    # left_hip_roll
    [-1.57, 1.57],    # left_hip_pitch
    [-0.26, 2.05],    # left_knee
    [-0.87, 0.52],    # left_ankle
    [-0.43, 0.43],    # right_hip_yaw
    [-0.43, 0.43],    # right_hip_roll
    [-1.57, 1.57],    # right_hip_pitch
    [-0.26, 2.05],    # right_knee
    [-0.87, 0.52],    # right_ankle
    [-2.35, 2.35],    # torso
    [-2.87, 2.87],    # left_shoulder_pitch
    [-0.34, 3.11],    # left_shoulder_roll
    [-1.3, 4.45],     # left_shoulder_yaw
    [-1.25, 2.61],    # left_elbow
    [-2.87, 2.87],    # right_shoulder_pitch
    [-3.11, 0.34],    # right_shoulder_roll
    [-4.45, 1.3],     # right_shoulder_yaw
    [-1.25, 2.61]     # right_elbow
])

# Max continuous velocity (rad/s)
VELOCITY_LIMITS = np.ones(NUM_JOINTS) * 5.0 # default general cap for all joints

# Max torque (Nm) limits from H1 datasheet
TORQUE_LIMITS = np.array([
    200, 200, 200, 300, 40,   # Left leg
    200, 200, 200, 300, 40,   # Right leg
    200,                      # Torso
    40, 40, 18, 18,           # Left arm
    40, 40, 18, 18            # Right arm
])
