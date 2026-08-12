from .sim_backend import SimBackend
from .real_backend import RealBackend

def create_backend(config: dict):
    backend_type = config.get("BACKEND", "sim")
    if backend_type == "sim":
        return SimBackend()
    elif backend_type == "real":
        return RealBackend()
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")
