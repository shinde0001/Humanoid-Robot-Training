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
    return {"status": "override_cleared"}

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if sim_instance:
                state = sim_instance.backend.get_state()
                pending = sim_instance.validation_gate.pending_command
                data = {
                    "timestamp": state.timestamp,
                    "joints": state.joint_positions.tolist(),
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
                    # Request low-res frame to save bandwidth
                    img = sim_instance.backend.render_camera("head", 320, 240)
                    if img is not None:
                        # MuJoCo returns RGB, cv2 expects BGR
                        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        _, buffer = cv2.imencode('.jpg', img_bgr)
                        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                        await websocket.send_text(jpg_as_text)
                except NotImplementedError:
                    pass
                except Exception as e:
                    pass
            await asyncio.sleep(1 / 15.0) # 15 Hz for video stream
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
