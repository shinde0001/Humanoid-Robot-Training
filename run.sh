#!/bin/bash
# run.sh - Master launcher for Project PARTH
# Boots the Python simulation engine and the unified J.A.R.V.I.S. dashboard server.

set -e

echo "=========================================="
echo "    INITIALIZING PROJECT PARTH            "
echo "=========================================="

# Ensure we are in the correct directory
cd "$(dirname "$0")"

# 1. Clean up any previous dangling processes on port 8000 or 8080
lsof -ti:8000,8080 | xargs kill -9 2>/dev/null || true

# 2. Boot the core simulation engine and web dashboard on port 8000
echo "[*] Booting MuJoCo Engine & J.A.R.V.I.S. Dashboard on http://localhost:8000..."
exec python3 main.py
