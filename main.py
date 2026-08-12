import config
from hal.factory import create_backend
from core.simulation import CoreSimulation
import threading
from dashboard.backend.server import run_server

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
    
    # Launch Dashboard API Server in a background thread
    api_thread = threading.Thread(target=run_server, args=(sim, "0.0.0.0", 8000), daemon=True)
    api_thread.start()
    print("Dashboard API Server running on ws://0.0.0.0:8000")
    
    try:
        print("Running simulation loop. Press Ctrl+C to stop.")
        # Run indefinitely (or until E-Stop kills it)
        sim.step_loop(num_steps=None)
    except KeyboardInterrupt:
        print("Interrupted by user. Shutting down...")
        sim.stop()
    
    print("System Shutdown Complete.")

if __name__ == "__main__":
    main()
