import time
import numpy as np
import mujoco
from .interface import RobotBackend, RobotState, IMUData

class SimBackend(RobotBackend):
    def __init__(self):
        self.model = None
        self.data = None
        self.renderer = None
        self.target_yaw = 0.0
        self.nav_vx = 0.0
        self.nav_vy = 0.0
        self.nav_vyaw = 0.0
        self.target_height = 0.98
        self.current_mode = "stand"
        
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
        
    def set_navigation_targets(self, mode: str, vx: float, vy: float, vyaw: float, target_height: float = 0.98) -> None:
        """Update navigation velocity and target height from control/gait engine"""
        self.current_mode = mode
        self.nav_vx = vx
        self.nav_vy = vy
        self.nav_vyaw = vyaw
        self.target_height = target_height
        
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
        pelvis_id = self.model.body("pelvis").id
        dt = self.model.opt.timestep
        
        # Integrate heading angle when turning command is active
        if abs(self.nav_vyaw) > 1e-4:
            self.target_yaw += self.nav_vyaw * dt
            
        # Target heading quaternion [w, 0, 0, z]
        w_t = np.cos(self.target_yaw / 2.0)
        z_t = np.sin(self.target_yaw / 2.0)
        
        # Orientation stabilizer to commanded heading
        quat = self.data.qpos[3:7] # [w, x, y, z]
        err_rot = 2.0 * (quat[1:4] * w_t - np.array([0.0, 0.0, z_t]) * quat[0] - np.cross(quat[1:4], [0.0, 0.0, z_t]))
        ang_vel = self.data.qvel[3:6]
        torque_assist = -500.0 * err_rot - 50.0 * ang_vel
        
        # Height spring-damper to maintain commanded target height
        z = self.data.qpos[2]
        vz = self.data.qvel[2]
        force_z = -1500.0 * (z - self.target_height) - 150.0 * vz + 51.4 * 9.81
        
        # Direction vectors based on current heading
        dir_x = np.cos(self.target_yaw)
        dir_y = np.sin(self.target_yaw)
        lat_x = -np.sin(self.target_yaw)
        lat_y = np.cos(self.target_yaw)
        
        # Planar propulsion in heading direction
        force_x = (dir_x * self.nav_vx + lat_x * self.nav_vy) * 60.0
        force_y = (dir_y * self.nav_vx + lat_y * self.nav_vy) * 60.0
        
        self.data.xfrc_applied[pelvis_id, :3] = np.array([force_x, force_y, max(0.0, force_z)])
        self.data.xfrc_applied[pelvis_id, 3:] = torque_assist
        
        mujoco.mj_step(self.model, self.data)

    def emergency_stop(self) -> None:
        self.data.ctrl[:] = 0.0 # Zero all torques
        self.nav_vx = 0.0
        self.nav_vy = 0.0
        self.nav_vyaw = 0.0
        pelvis_id = self.model.body("pelvis").id
        self.data.xfrc_applied[pelvis_id, :] = 0.0

    def shutdown(self) -> None:
        if self.renderer:
            self.renderer.close()
