# config.py
import os

# Set BACKEND to "sim" for MuJoCo simulation, or "real" for hardware
BACKEND = os.environ.get("PARTH_BACKEND", "sim")

# Simulation config
SIM_MODEL_PATH = "models/parth_humanoid/scene.xml"
CONTROL_RATE_HZ = 200
TIMESTEP = 1.0 / CONTROL_RATE_HZ

# Safety config
WATCHDOG_TIMEOUT_MS = 50
