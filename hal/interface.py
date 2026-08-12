from abc import ABC, abstractmethod
import numpy as np
from dataclasses import dataclass

@dataclass
class IMUData:
    """Inertial measurement data."""
    orientation: np.ndarray    # quaternion [w, x, y, z]
    angular_velocity: np.ndarray  # [wx, wy, wz] rad/s
    linear_acceleration: np.ndarray  # [ax, ay, az] m/s^2

@dataclass
class RobotState:
    """Complete robot state snapshot."""
    timestamp: float
    joint_positions: np.ndarray    # radians
    joint_velocities: np.ndarray   # rad/s
    joint_torques: np.ndarray      # Nm
    imu: IMUData
    contact_forces: np.ndarray     # per-foot [left_z, right_z] Newtons
    battery_voltage: float         # Volts

class RobotBackend(ABC):
    """Abstract interface for all hardware and simulation backends."""
    
    @abstractmethod
    def initialize(self, config: dict) -> None:
        """Initialize connection to the robot or simulation."""
        pass

    @abstractmethod
    def get_state(self) -> RobotState:
        """Read the current state of the robot."""
        pass

    @abstractmethod
    def send_commands(self, joint_torques: np.ndarray) -> None:
        """Send torque commands to the robot actuators."""
        pass

    @abstractmethod
    def render_camera(self, eye: str, width: int, height: int) -> np.ndarray:
        """Render a camera frame as a numpy array."""
        pass

    @abstractmethod
    def step(self) -> None:
        """Advance the simulation (or wait for the next control tick on real hardware)."""
        pass

    @abstractmethod
    def emergency_stop(self) -> None:
        """Instantly halt the robot safely."""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Cleanly shutdown the robot or simulation."""
        pass
