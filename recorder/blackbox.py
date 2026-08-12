# recorder/blackbox.py
import numpy as np
import os
import time
from typing import Optional
from hal.interface import RobotState
import logging

class DataRecorder:
    """
    Tier 0: Always-on flight data recorder (Black Box).
    Stores exactly `history_seconds` worth of data in a fast, pre-allocated NumPy ring buffer.
    """
    def __init__(self, rate_hz: int, history_seconds: int = 60, save_dir: str = "data/recordings"):
        self.rate_hz = rate_hz
        self.history_seconds = history_seconds
        self.max_frames = rate_hz * history_seconds
        self.save_dir = save_dir
        
        # Ensure save directory exists
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Pre-allocate ring buffers
        # 19 actuated joints
        self.timestamps = np.zeros(self.max_frames, dtype=np.float64)
        self.positions = np.zeros((self.max_frames, 19), dtype=np.float64)
        self.velocities = np.zeros((self.max_frames, 19), dtype=np.float64)
        self.torques = np.zeros((self.max_frames, 19), dtype=np.float64)
        
        self.current_idx = 0
        self.total_recorded = 0
        self.logger = logging.getLogger("DataRecorder")
        
    def record_tick(self, state: RobotState) -> None:
        """Called every single control loop to log data."""
        idx = self.current_idx
        
        self.timestamps[idx] = state.timestamp
        self.positions[idx] = state.joint_positions
        self.velocities[idx] = state.joint_velocities
        self.torques[idx] = state.joint_torques
        
        self.current_idx = (self.current_idx + 1) % self.max_frames
        if self.total_recorded < self.max_frames:
            self.total_recorded += 1
            
    def dump_to_disk(self, filename: Optional[str] = None) -> str:
        """
        Extracts chronological data from the ring buffer and saves to .npz
        """
        if filename is None:
            filename = f"flight_data_{int(time.time())}.npz"
            
        filepath = os.path.join(self.save_dir, filename)
        
        if self.total_recorded == 0:
            self.logger.warning("No data to dump.")
            return filepath
            
        # Re-align ring buffer chronologically
        if self.total_recorded < self.max_frames:
            # We haven't looped yet
            valid_idx = slice(0, self.current_idx)
            ts = self.timestamps[valid_idx]
            pos = self.positions[valid_idx]
            vel = self.velocities[valid_idx]
            trq = self.torques[valid_idx]
        else:
            # Buffer wrapped, chronological order is from current_idx to end, then 0 to current_idx
            ts = np.concatenate((self.timestamps[self.current_idx:], self.timestamps[:self.current_idx]))
            pos = np.concatenate((self.positions[self.current_idx:], self.positions[:self.current_idx]))
            vel = np.concatenate((self.velocities[self.current_idx:], self.velocities[:self.current_idx]))
            trq = np.concatenate((self.torques[self.current_idx:], self.torques[:self.current_idx]))
            
        np.savez_compressed(
            filepath,
            timestamps=ts,
            positions=pos,
            velocities=vel,
            torques=trq,
            rate_hz=self.rate_hz
        )
        self.logger.info(f"Black box data dumped to {filepath}")
        return filepath
