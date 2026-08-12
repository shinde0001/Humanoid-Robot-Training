# command/parser.py
from dataclasses import dataclass
from typing import Optional
import json
import logging

@dataclass
class RobotCommand:
    type: str  # 'stand', 'walk', 'crouch', 'estop'
    v_x: float = 0.0
    v_y: float = 0.0
    v_yaw: float = 0.0
    
class CommandParser:
    def __init__(self):
        self.logger = logging.getLogger("CommandParser")
        
    def parse(self, raw_json_str: str) -> Optional[RobotCommand]:
        try:
            data = json.loads(raw_json_str)
            cmd_type = data.get("type")
            
            VALID_COMMANDS = {
                'stand', 'walk', 'crouch', 'jump', 'wave', 'punch', 
                'victory', 'fight', 'look_left', 'look_right', 'scan', 
                'bow', 'estop', 'zero'
            }
            if cmd_type not in VALID_COMMANDS:
                self.logger.warning(f"Unknown command type: {cmd_type}")
                return None
                
            return RobotCommand(
                type=cmd_type,
                v_x=float(data.get("v_x", 0.0)),
                v_y=float(data.get("v_y", 0.0)),
                v_yaw=float(data.get("v_yaw", 0.0))
            )
        except Exception as e:
            self.logger.error(f"Failed to parse command: {e}")
            return None
