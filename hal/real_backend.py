import numpy as np
from .interface import RobotBackend, RobotState, IMUData

class RealBackend(RobotBackend):
    def initialize(self, config: dict) -> None:
        raise NotImplementedError("Real hardware backend is not yet implemented. Please use BACKEND='sim' in config.py")

    def get_state(self) -> RobotState:
        raise NotImplementedError()

    def send_commands(self, joint_torques: np.ndarray) -> None:
        raise NotImplementedError()

    def render_camera(self, eye: str, width: int, height: int) -> np.ndarray:
        raise NotImplementedError()

    def step(self) -> None:
        raise NotImplementedError()

    def emergency_stop(self) -> None:
        raise NotImplementedError()

    def shutdown(self) -> None:
        raise NotImplementedError()
