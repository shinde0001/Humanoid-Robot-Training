import time
import numpy as np
import mujoco
from .interface import RobotBackend, RobotState, IMUData

class SimBackend(RobotBackend):
    def __init__(self):
        self.model = None
        self.data = None
        self.renderer = None
        
    def initialize(self, config: dict) -> None:
        model_path = config.get("model_path", "models/parth_humanoid/scene.xml")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Configure simulation timestep
        self.model.opt.timestep = config.get("timestep", 0.005) # 200Hz
        mujoco.mj_forward(self.model, self.data)
        
        # We can initialize the renderer lazily when render_camera is called
        
    def get_state(self) -> RobotState:
        # For H1, qpos has length 26 (7 free joint + 19 actuated)
        # qvel has length 25 (6 free joint + 19 actuated)
        # We'll just extract the actuated ones for simplicity or return all
        # To make it simple for Phase 1, we return the full arrays
        
        # Note: real IMU mapping depends on the MJCF sensor definitions.
        # We will map this properly in later phases.
        imu_data = IMUData(
            orientation=np.array([1, 0, 0, 0]), 
            angular_velocity=np.zeros(3),
            linear_acceleration=np.zeros(3)
        )
        
        state = RobotState(
            timestamp=self.data.time,
            joint_positions=self.data.qpos.copy(),
            joint_velocities=self.data.qvel.copy(),
            joint_torques=self.data.ctrl.copy(),
            imu=imu_data,
            contact_forces=np.zeros(2),
            battery_voltage=48.0
        )
        return state

    def send_commands(self, joint_torques: np.ndarray) -> None:
        # H1 model in menagerie has 19 actuators
        if len(joint_torques) == self.model.nu:
            self.data.ctrl[:] = joint_torques

    def render_camera(self, eye: str, width: int, height: int) -> np.ndarray:
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height, width)
        
        self.renderer.update_scene(self.data)
        return self.renderer.render()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def emergency_stop(self) -> None:
        self.data.ctrl[:] = 0.0 # Zero all torques

    def shutdown(self) -> None:
        if self.renderer:
            self.renderer.close()
