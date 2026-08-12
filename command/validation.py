import logging
from typing import Optional
from .parser import RobotCommand

class ValidationGate:
    """
    Tier 0 Safety: Pauses any AI or external commands until explicitly approved by a human.
    """
    def __init__(self):
        self.pending_command: Optional[RobotCommand] = None
        self.logger = logging.getLogger("ValidationGate")
        
    def propose_command(self, cmd: RobotCommand) -> None:
        """Stores a command in the pending queue. Does NOT execute it."""
        self.logger.info(f"Command proposed and paused for validation: {cmd.type}")
        self.pending_command = cmd
        
    def approve_command(self) -> Optional[RobotCommand]:
        """Human clicks 'Approve' -> releases the command for execution."""
        if self.pending_command:
            cmd = self.pending_command
            self.logger.warning(f"HUMAN APPROVED COMMAND: {cmd.type}")
            self.pending_command = None
            return cmd
        return None
        
    def reject_command(self) -> None:
        """Human clicks 'Reject' -> drops the command."""
        if self.pending_command:
            self.logger.warning(f"HUMAN REJECTED COMMAND: {self.pending_command.type}")
            self.pending_command = None

class ManualOverride:
    """
    Tier 0 Safety: Keyboard/Joystick direct control that preempts all other logic.
    """
    def __init__(self):
        self.active_override: Optional[RobotCommand] = None
        self.logger = logging.getLogger("ManualOverride")
        
    def set_override(self, cmd: RobotCommand) -> None:
        """Instantly forces this command over any AI logic."""
        self.active_override = cmd
        self.logger.warning(f"MANUAL OVERRIDE ENGAGED: {cmd.type}")
        
    def clear_override(self) -> None:
        """Releases manual control back to the autonomous/gated system."""
        if self.active_override:
            self.logger.info("Manual override released.")
            self.active_override = None
            
    def get_override(self) -> Optional[RobotCommand]:
        return self.active_override
