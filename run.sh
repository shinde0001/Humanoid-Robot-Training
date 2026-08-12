#!/bin/bash
# run.sh - Master launcher for Project PARTH
# Boots the Python simulation engine and the HTML dashboard server.

set -e

echo "=========================================="
echo "    INITIALIZING PROJECT PARTH            "
echo "=========================================="

# Ensure we are in the correct directory
cd "$(dirname "$0")"

# 1. Start the frontend dashboard server in the background
echo "[1/2] Starting J.A.R.V.I.S. Dashboard Server on port 8080..."
python3 -m http.server 8080 --directory dashboard/frontend &
FRONTEND_PID=$!

# 2. Start the core simulation engine and API server
echo "[2/2] Booting MuJoCo Simulation Engine & Hardware Abstraction Layer..."
sleep 1
python3 main.py

# Cleanup when main.py exits
echo "Simulation Engine terminated. Shutting down Dashboard Server..."
kill $FRONTEND_PID
wait $FRONTEND_PID 2>/dev/null || true
echo "Shutdown complete. Goodbye."
