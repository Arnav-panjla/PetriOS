#!/usr/bin/env bash
# One command: SSH tunnel to the lab GPU (Ollama / Qwen), then the web GUI with vision labels on.
#
#   ./start_vision.sh                  # GUI in simulation-only mode (no arm motion)
#   ./start_vision.sh --port 8001      # extra arguments go to the GUI (replaces the default)
#   ./start_vision.sh --webcam-rotate 90 --simulation-only
#
# Override with environment variables: GPU_USER, GPU_HOST, LOCAL_PORT, VLM_MODEL.
# The Pi must be on the IITD network or VPN. You will be asked for the lab password once.

set -euo pipefail

GPU_USER="${GPU_USER:-ee1230978}"
GPU_HOST="${GPU_HOST:-10.225.67.239}"
LOCAL_PORT="${LOCAL_PORT:-11435}"
REMOTE_PORT=11434
SOCK="/tmp/vision_tunnel_${LOCAL_PORT}.sock"

cd "$(dirname "$0")"
# shellcheck disable=SC1091
source .venv/bin/activate

stop_tunnel() { ssh -S "$SOCK" -O exit "$GPU_USER@$GPU_HOST" 2>/dev/null || true; }
trap stop_tunnel EXIT

# Clear anything left over from an earlier run (a stale tunnel was our hang before).
stop_tunnel
pkill -f robot_arm.interfaces.web_gui 2>/dev/null || true
rm -f "$SOCK"

echo "Opening tunnel to $GPU_USER@$GPU_HOST (local port $LOCAL_PORT) ..."
ssh -M -S "$SOCK" -f -N \
    -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
    -L "$LOCAL_PORT:127.0.0.1:$REMOTE_PORT" "$GPU_USER@$GPU_HOST"

echo -n "Checking that Qwen answers through the tunnel ... "
if ! tags="$(curl -s --noproxy '*' --max-time 10 "http://127.0.0.1:$LOCAL_PORT/api/tags")"; then
    echo "FAILED"
    echo "The tunnel is up but Ollama did not answer. Is 'ollama serve' running on the lab server?" >&2
    exit 1
fi
model="${VLM_MODEL:-qwen3.5:9b}"
if [[ "$tags" != *"$model"* ]]; then
    echo "reachable, but model '$model' is not on the server"
    echo "Models there: $tags" | head -c 300 >&2
    echo >&2
    exit 1
fi
echo "ok ($model)"

export OLLAMA_URL="http://127.0.0.1:$LOCAL_PORT"
args=("$@")
[[ ${#args[@]} -eq 0 ]] && args=(--simulation-only)

echo "Starting GUI: python -m robot_arm.interfaces.web_gui ${args[*]}"
python -m robot_arm.interfaces.web_gui "${args[@]}"
