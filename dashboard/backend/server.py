import asyncio
import base64
import cv2
import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
import uvicorn
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="PARTH Dashboard API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global reference to the core simulation engine
sim_instance = None

class CommandModel(BaseModel):
    type: str
    v_x: float = 0.0
    v_y: float = 0.0
    v_yaw: float = 0.0

@app.post("/api/command")
def propose_command(cmd: CommandModel):
    if sim_instance:
        sim_instance.propose_command_string(cmd.model_dump_json())
    return {"status": "proposed", "command": cmd.type}

@app.post("/api/approve")
def approve_command():
    if sim_instance:
        sim_instance.approve_pending_command()
    return {"status": "approved"}

@app.post("/api/reject")
def reject_command():
    if sim_instance:
        sim_instance.validation_gate.reject_command()
    return {"status": "rejected"}

@app.post("/api/override")
def manual_override(cmd: CommandModel):
    if sim_instance:
        sim_instance.force_manual_command(cmd.type, cmd.v_x, cmd.v_y, cmd.v_yaw)
    return {"status": "overridden", "command": cmd.type}

@app.post("/api/clear_override")
def clear_override():
    if sim_instance:
        sim_instance.manual_override.clear_override()
        sim_instance.gait_engine.set_command('stand', {})
    return {"status": "override_cleared"}

import numpy as np
from safety.limits import TORQUE_LIMITS, POSITION_LIMITS, VELOCITY_LIMITS, JOINT_NAMES

@app.post("/api/reset_estop")
@app.post("/api/release_estop")
def reset_estop():
    if sim_instance:
        sim_instance.reset_estop()
    return {"status": "estop_released"}

@app.get("/api/diagnostics")
@app.post("/api/diagnostics/run")
def run_motor_diagnostics():
    if not sim_instance:
        return {"status": "error", "message": "Simulation offline"}
    state = sim_instance.backend.get_state()
    report = []
    all_healthy = True
    for i, name in enumerate(JOINT_NAMES):
        pos = float(state.joint_positions[i]) if i < len(state.joint_positions) else 0.0
        vel = float(state.joint_velocities[i]) if i < len(state.joint_velocities) else 0.0
        torque = float(state.joint_torques[i]) if i < len(state.joint_torques) else 0.0
        pos_min, pos_max = float(POSITION_LIMITS[i][0]), float(POSITION_LIMITS[i][1])
        t_limit = float(TORQUE_LIMITS[i])
        
        pos_ok = (pos_min - 0.05 <= pos <= pos_max + 0.05)
        t_ok = abs(torque) <= (t_limit + 1.0)
        healthy = pos_ok and t_ok and not np.isnan(pos) and not np.isnan(torque)
        if not healthy:
            all_healthy = False
            
        report.append({
            "id": i,
            "name": name,
            "healthy": healthy,
            "status": "NOMINAL" if healthy else "FAULT",
            "position": round(pos, 3),
            "velocity": round(vel, 3),
            "torque": round(torque, 2),
            "torque_limit": t_limit,
            "pos_limits": [pos_min, pos_max],
            "load_pct": min(100.0, round(abs(torque) / t_limit * 100, 1)) if t_limit > 0 else 0,
            "temp_c": round(32.0 + abs(torque) * 0.12 + abs(vel) * 0.5, 1)
        })
    return {
        "status": "complete",
        "all_healthy": all_healthy,
        "estopped": sim_instance.safety.is_estopped(),
        "total_motors": len(JOINT_NAMES),
        "online_motors": sum(1 for m in report if m["healthy"]),
        "report": report
    }

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if sim_instance:
                state = sim_instance.backend.get_state()
                pending = sim_instance.validation_gate.pending_command
                
                # Compute Euler angles from floating base pelvis quaternion [w, x, y, z]
                roll_deg, pitch_deg, yaw_deg = 0.0, 0.0, 0.0
                if state.imu and len(state.imu.orientation) == 4:
                    w, x, y, z = state.imu.orientation
                    sinr_cosp = 2.0 * (w * x + y * z)
                    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
                    roll_deg = float(np.degrees(np.arctan2(sinr_cosp, cosr_cosp)))
                    
                    sinp = 2.0 * (w * y - z * x)
                    if abs(sinp) >= 1:
                        pitch_deg = float(np.degrees(np.copysign(np.pi / 2, sinp)))
                    else:
                        pitch_deg = float(np.degrees(np.arcsin(sinp)))
                        
                    siny_cosp = 2.0 * (w * z + x * y)
                    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                    yaw_deg = float(np.degrees(np.arctan2(siny_cosp, cosy_cosp)))
                
                # Build rich real-time actuator telemetry for the Holographic Blueprint
                motor_telemetry = []
                n_act = min(len(JOINT_NAMES), len(state.joint_positions))
                for i in range(n_act):
                    pos = float(state.joint_positions[i])
                    vel = float(state.joint_velocities[i]) if i < len(state.joint_velocities) else 0.0
                    torq = float(state.joint_torques[i]) if i < len(state.joint_torques) else 0.0
                    p_min, p_max = float(POSITION_LIMITS[i][0]), float(POSITION_LIMITS[i][1])
                    t_max = float(TORQUE_LIMITS[i])
                    
                    pos_ok = (p_min - 0.05 <= pos <= p_max + 0.05)
                    torq_ok = abs(torq) <= (t_max + 1.0)
                    is_healthy = pos_ok and torq_ok and not np.isnan(pos) and not np.isnan(torq)
                    load_pct = min(100.0, abs(torq) / t_max * 100.0) if t_max > 0 else 0.0
                    
                    motor_telemetry.append({
                        "id": i,
                        "name": JOINT_NAMES[i],
                        "pos": round(pos, 3),
                        "pos_deg": round(float(np.degrees(pos)), 1),
                        "vel": round(vel, 3),
                        "torque": round(torq, 2),
                        "torque_limit": t_max,
                        "pos_limits": [p_min, p_max],
                        "load_pct": round(load_pct, 1),
                        "temp_c": round(32.0 + abs(torq) * 0.12 + abs(vel) * 0.5, 1),
                        "healthy": is_healthy
                    })
                
                data = {
                    "timestamp": state.timestamp,
                    "joints": state.joint_positions.tolist(),
                    "joint_velocities": state.joint_velocities.tolist(),
                    "joint_torques": state.joint_torques.tolist(),
                    "motors": motor_telemetry,
                    "battery_voltage": getattr(state, 'battery_voltage', 48.0),
                    "contact_forces": state.contact_forces.tolist() if hasattr(state, 'contact_forces') else [0.0, 0.0],
                    "imu": {
                        "roll": round(roll_deg, 1),
                        "pitch": round(pitch_deg, 1),
                        "yaw": round(yaw_deg, 1),
                        "ang_vel": state.imu.angular_velocity.tolist() if state.imu else [0, 0, 0]
                    },
                    "state": sim_instance.gait_engine.current_state,
                    "estopped": sim_instance.safety.is_estopped(),
                    "pending_command": pending.type if pending else None,
                    "manual_override": sim_instance.manual_override.get_override() is not None
                }
                await websocket.send_text(json.dumps(data))
            await asyncio.sleep(1 / 30.0) # 30 Hz stream
    except WebSocketDisconnect:
        pass

@app.websocket("/ws/video")
async def websocket_video(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if sim_instance:
                try:
                    img = sim_instance.backend.render_camera("egocentric", 320, 240)
                    if img is not None:
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        _, buffer = cv2.imencode('.jpg', img_bgr)
                        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                        await websocket.send_text(jpg_as_text)
                except Exception:
                    pass
            await asyncio.sleep(1 / 15.0)
    except WebSocketDisconnect:
        pass

@app.websocket("/ws/video_cinematic")
async def websocket_video_cinematic(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if sim_instance:
                try:
                    img = sim_instance.backend.render_camera("cinematic", 320, 240)
                    if img is not None:
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        _, buffer = cv2.imencode('.jpg', img_bgr)
                        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                        await websocket.send_text(jpg_as_text)
                except Exception:
                    pass
            await asyncio.sleep(1 / 15.0)
    except WebSocketDisconnect:
        pass

# Mount frontend directory for direct dashboard access
import os
from fastapi.staticfiles import StaticFiles
frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend"))
if os.path.exists(frontend_dir):
    app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")

def run_server(sim, host="0.0.0.0", port=8000):
    global sim_instance
    sim_instance = sim
    uvicorn.run(app, host=host, port=port, log_level="warning")
