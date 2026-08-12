import config
from hal.factory import create_backend
from core.simulation import CoreSimulation

def main():
    print(f"--- Project PARTH Simulation ---")
    print(f"Configured Backend: {config.BACKEND}")
    
    cfg = {
        "BACKEND": config.BACKEND,
        "model_path": config.SIM_MODEL_PATH,
        "CONTROL_RATE_HZ": config.CONTROL_RATE_HZ,
        "timestep": config.TIMESTEP
    }
    
    backend = create_backend(cfg)
    sim = CoreSimulation(backend, cfg)
    
    sim.start()
    
    print("Running 1000 steps test...")
    sim.step_loop(num_steps=1000)
    
    print("Phase 1 test complete.")

if __name__ == "__main__":
    main()
