import time
import logging

class CoreSimulation:
    def __init__(self, backend, config):
        self.backend = backend
        self.config = config
        self.rate_hz = config.get("CONTROL_RATE_HZ", 200)
        self.running = False
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger("SimulationCore")

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
        target_dt = 1.0 / self.rate_hz
        
        while self.running:
            tick_start = time.perf_counter()
            
            # 1. READ
            state = self.backend.get_state()
            
            # (Phase 1: Just logging state occasionally)
            if step_count % 1000 == 0:
                self.logger.info(f"Step {step_count} | Sim Time: {state.timestamp:.3f}s | Torques: {state.joint_torques.shape}")
                
            # 2-4: Skipping Safety and Control for Phase 1 (to be added in later phases)
            
            # 5. ACT
            # In phase 1, we just apply zero torques or let MuJoCo handle its default PD controllers
            # Note: The Menagerie H1 uses position/velocity actuators, we will refine this.
            
            # 6. STEP
            self.backend.step()
            
            # 7. PACE (Only strictly necessary if we want real-time pacing in sim)
            # In purely simulated tests, we can run faster than real-time.
            # elapsed = time.perf_counter() - tick_start
            # sleep_time = target_dt - elapsed
            # if sleep_time > 0:
            #     time.sleep(sleep_time)
                
            step_count += 1
            if num_steps is not None and step_count >= num_steps:
                self.stop()
                break
