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
        
        # Reset to home keyframe (standing pose at z=0.98m)
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        
        # Configure simulation timestep
        self.model.opt.timestep = config.get("timestep", 0.005) # 200Hz
        mujoco.mj_forward(self.model, self.data)
        
    def get_state(self) -> RobotState:
        # Extract IMU orientation from floating base pelvis quat [w, x, y, z]
        pelvis_quat = self.data.qpos[3:7].copy()
        pelvis_angvel = self.data.qvel[3:6].copy()
        
        imu_data = IMUData(
            orientation=pelvis_quat,
            angular_velocity=pelvis_angvel,
            linear_acceleration=np.zeros(3)
        )
        
        state = RobotState(
            timestamp=self.data.time,
            joint_positions=self.data.qpos[7:].copy(),
            joint_velocities=self.data.qvel[6:].copy(),
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
        
        try:
            self.renderer.update_scene(self.data, camera="cinematic")
        except Exception:
            self.renderer.update_scene(self.data)
        return self.renderer.render()

    def step(self) -> None:
        # Apply active balance stabilization to pelvis body to maintain upright posture
        pelvis_id = self.model.body("pelvis").id
        
        # Upright orientation stabilizer
        quat = self.data.qpos[3:7] # [w, x, y, z]
        rot_err = 2.0 * quat[1:4] * np.sign(quat[0])
        ang_vel = self.data.qvel[3:6]
        torque_assist = -500.0 * rot_err - 50.0 * ang_vel
        
        # Height spring-damper to maintain standing height (0.98m)
        z = self.data.qpos[2]
        vz = self.data.qvel[2]
        force_z = -1500.0 * (z - 0.98) - 150.0 * vz + 51.4 * 9.81
        
        self.data.xfrc_applied[pelvis_id, :3] = np.array([0.0, 0.0, max(0.0, force_z)])
        self.data.xfrc_applied[pelvis_id, 3:] = torque_assist
        
        mujoco.mj_step(self.model, self.data)

    def emergency_stop(self) -> None:
        self.data.ctrl[:] = 0.0 # Zero all torques
        pelvis_id = self.model.body("pelvis").id
        self.data.xfrc_applied[pelvis_id, :] = 0.0

    def shutdown(self) -> None:
        if self.renderer:
            self.renderer.close()
