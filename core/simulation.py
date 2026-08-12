import time
import logging
import numpy as np
from safety.safety_controller import SafetyController

class CoreSimulation:
    def __init__(self, backend, config):
        self.backend = backend
        self.config = config
        self.rate_hz = config.get("CONTROL_RATE_HZ", 200)
        self.dt = 1.0 / self.rate_hz
        self.running = False
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("SimulationCore")
        
        # Initialize Tier 1 & 2 Safety
        self.safety = SafetyController(self.dt)

    def start(self):
        self.backend.initialize(self.config)
        self.running = True
        self.logger.info(f"Started simulation loop at {self.rate_hz} Hz with backend: {self.config.get('BACKEND')}")
        
    def stop(self):
        self.running = False
        self.backend.shutdown()
        self.logger.info("Simulation stopped.")
        
    def step_loop(self, num_steps=None):
        step_count = 0
        target_dt = self.dt
        
        while self.running:
            tick_start = time.perf_counter()
            
            # 1. READ
            state = self.backend.get_state()
            
            # 2. SAFETY CHECK (Tier 2)
            self.safety.check_state(state)
            
            # (Phase 1/2: Just logging state occasionally)
            if step_count % 1000 == 0:
                self.logger.info(f"Step {step_count} | Sim Time: {state.timestamp:.3f}s | Torques: {state.joint_torques.shape}")
                
            # 3. THINK (Placeholder for Controller)
            # We request 0 torques, or we can test an over-limit torque here.
            raw_command_torques = np.zeros(19)
            
            # 4. SAFETY CLAMP (Tier 1)
            safe_torques = self.safety.check_and_clamp_torques(
                raw_command_torques, current_positions=state.joint_positions
            )
            
            # If E-Stop triggered, HAL handles immediate stop
            if self.safety.is_estopped():
                self.backend.emergency_stop()
                self.logger.critical("E-STOP active. Halting loop.")
                self.stop()
                break
            
            # 5. ACT
            self.backend.send_commands(safe_torques)
            
            # 6. STEP
            self.backend.step()
                
            step_count += 1
            if num_steps is not None and step_count >= num_steps:
                self.stop()
                break
