import time
import logging
import numpy as np
from safety.safety_controller import SafetyController
from control.pid import PDController
from control.gait_engine import GaitEngine
from command.parser import CommandParser, RobotCommand
from command.validation import ValidationGate, ManualOverride
from recorder.blackbox import DataRecorder

class CoreSimulation:
    def __init__(self, backend, config):
        self.backend = backend
        self.config = config
        self.rate_hz = config.get("CONTROL_RATE_HZ", 200)
        self.dt = 1.0 / self.rate_hz
        self.running = False
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("SimulationCore")
        
        # Initialize Safety & Validation
        self.safety = SafetyController(self.dt)
        self.validation_gate = ValidationGate()
        self.manual_override = ManualOverride()
        
        # Initialize Recorder
        self.recorder = DataRecorder(self.rate_hz, history_seconds=60)
        
        # Initialize Control with tuned joint gains
        kp = np.array([200, 200, 350, 350, 150,  200, 200, 350, 350, 150,  200,  80, 80, 40, 40,  80, 80, 40, 40], dtype=float)
        kd = np.array([20, 20, 30, 30, 15,  20, 20, 30, 30, 15,  20,  10, 10, 5, 5,  10, 10, 5, 5], dtype=float)
        self.controller = PDController(kp, kd)
        self.gait_engine = GaitEngine(self.dt)
        self.parser = CommandParser()

    def start(self):
        self.backend.initialize(self.config)
        self.running = True
        self.logger.info(f"Started simulation loop at {self.rate_hz} Hz with backend: {self.config.get('BACKEND')}")
        
    def stop(self):
        self.running = False
        self.backend.shutdown()
        self.logger.info("Simulation stopped.")
        
    def propose_command_string(self, json_str: str):
        """Used by AI to propose a command (goes to Validation Gate)."""
        cmd = self.parser.parse(json_str)
        if cmd:
            if cmd.type == 'estop':
                self.safety.trigger_estop("AI E-STOP request.")
            else:
                self.validation_gate.propose_command(cmd)

    def force_manual_command(self, cmd_type: str, v_x: float=0.0, v_y: float=0.0, v_yaw: float=0.0):
        """Used by Joystick/Keyboard to bypass AI and Validation entirely."""
        if cmd_type == 'estop':
            self.safety.trigger_estop("MANUAL E-STOP HIT!")
        else:
            self.manual_override.set_override(RobotCommand(type=cmd_type, v_x=v_x, v_y=v_y, v_yaw=v_yaw))

    def approve_pending_command(self):
        """Called by Dashboard UI when human clicks 'Approve'."""
        cmd = self.validation_gate.approve_command()
        if cmd:
            self.gait_engine.set_command(cmd.type, {"v_x": cmd.v_x, "v_y": cmd.v_y, "v_yaw": cmd.v_yaw})

    def reset_estop(self):
        """Called by Dashboard UI to release E-STOP and restore upright standing stance."""
        self.safety.reset_estop()
        self.manual_override.clear_override()
        self.gait_engine.set_command('stand', {})
        if hasattr(self.backend, "reset_to_home"):
            self.backend.reset_to_home()
        self._crash_dumped = False
        self.logger.info("E-STOP released. Robot restored to safe standing stance.")

    def step_loop(self, num_steps=None):
        step_count = 0
        target_dt = self.dt
        self._crash_dumped = False
        
        while self.running:
            tick_start = time.perf_counter()
            
            # 1. READ
            state = self.backend.get_state()
            
            # 2. SAFETY CHECK (Tier 2)
            self.safety.check_state(state)
            
            # Logging
            if step_count % 1000 == 0:
                self.logger.info(f"Step {step_count} | Sim Time: {state.timestamp:.3f}s | Gait State: {self.gait_engine.current_state}")
                
            # Check E-STOP state
            if self.safety.is_estopped():
                self.backend.emergency_stop()
                if not self._crash_dumped:
                    self.logger.critical("E-STOP active. Robot immobilized in safe holding mode.")
                    self.recorder.dump_to_disk("crash_report.npz")
                    self._crash_dumped = True
                
                # In batch test mode, exit loop
                if num_steps is not None:
                    self.stop()
                    break
                
                # In live mode, step physics with zero torques and pace loop
                self.backend.step()
                elapsed = time.perf_counter() - tick_start
                sleep_time = target_dt - elapsed
                if sleep_time > 0.0005:
                    time.sleep(sleep_time)
                step_count += 1
                continue
            else:
                self._crash_dumped = False
                
            # 3. THINK
            # Apply manual override if active, bypassing current gait state
            override = self.manual_override.get_override()
            if override:
                self.gait_engine.set_command(override.type, {"v_x": override.v_x, "v_y": override.v_y, "v_yaw": override.v_yaw})
                
            # Determine target kinematic pose from GaitEngine
            target_pos, target_vel = self.gait_engine.update()
            
            # Auto-clear one-shot overrides (jump, bow) once completed
            if override and override.type in ['jump', 'bow'] and self.gait_engine.current_state == 'stand':
                self.manual_override.clear_override()
            
            # Calculate torques via PD Control
            raw_command_torques = self.controller.compute(
                target_positions=target_pos,
                target_velocities=target_vel,
                current_positions=state.joint_positions,
                current_velocities=state.joint_velocities
            )
            
            # 4. SAFETY CLAMP (Tier 1)
            safe_torques = self.safety.check_and_clamp_torques(
                raw_command_torques, current_positions=state.joint_positions
            )
            
            # 5. ACT
            self.backend.send_commands(safe_torques)
            
            # Forward navigation velocities and target height to backend
            if hasattr(self.backend, "set_navigation_targets"):
                self.backend.set_navigation_targets(
                    mode=self.gait_engine.current_state,
                    vx=self.gait_engine.walk_speed_x,
                    vy=self.gait_engine.walk_speed_y,
                    vyaw=self.gait_engine.walk_speed_yaw,
                    target_height=self.gait_engine.target_height
                )
            
            # 7. RECORD DATA
            self.recorder.record_tick(state)
            
            # 8. STEP
            self.backend.step()
            
            # Real-time pacing (200 Hz)
            elapsed = time.perf_counter() - tick_start
            sleep_time = target_dt - elapsed
            if sleep_time > 0.0005:
                time.sleep(sleep_time)
                
            step_count += 1
            if num_steps is not None and step_count >= num_steps:
                self.logger.info("Simulation completed requested steps.")
                self.recorder.dump_to_disk("flight_data_normal.npz")
                self.stop()
                break
