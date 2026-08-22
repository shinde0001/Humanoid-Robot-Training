import time
from hal.factory import create_backend
from core.simulation import CoreSimulation

cfg = {
    "BACKEND": "sim",
    "model_path": "models/parth_humanoid/scene.xml",
    "CONTROL_RATE_HZ": 200,
    "timestep": 0.005
}

backend = create_backend(cfg)
sim = CoreSimulation(backend, cfg)
sim.start()

sim.force_manual_command('namaste')
print("Running namaste override...")
sim.step_loop(num_steps=10)
