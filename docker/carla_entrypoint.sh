#!/usr/bin/env bash
# carla_entrypoint.sh — locate and launch CarlaUnreal.sh inside the container
#
# CarlaSetup.sh may place the packaged binary at different paths depending on
# the build target and UE version. This script probes the known locations.

set -euo pipefail

CARLA_HOST="${CARLA_HOST:-0.0.0.0}"
CARLA_PORT="${CARLA_PORT:-2000}"
EXTRA_ARGS="${CARLA_EXTRA_ARGS:-}"

# ── Locate the binary ─────────────────────────────────────────────────────────
SEARCH_PATHS=(
    "/workspace/carla/Build/Package/CarlaUnreal.sh"
    "/workspace/carla/Build/Package/Linux/CarlaUnreal.sh"
    "/workspace/carla/Build/CarlaUnreal.sh"
    "/workspace/carla/CarlaUnreal.sh"
)

CARLA_BIN=""
for path in "${SEARCH_PATHS[@]}"; do
    if [[ -f "$path" ]]; then
        CARLA_BIN="$path"
        break
    fi
done

if [[ -z "$CARLA_BIN" ]]; then
    echo "[carla_entrypoint] ERROR: CarlaUnreal.sh not found in known locations:"
    for path in "${SEARCH_PATHS[@]}"; do
        echo "  $path"
    done
    echo ""
    echo "  Has the build completed? Run:"
    echo "    cmake --build Build --target package"
    exit 1
fi

echo "[carla_entrypoint] Found binary: $CARLA_BIN"
echo "[carla_entrypoint] Host: $CARLA_HOST  Port: $CARLA_PORT"

# ── Vulkan ICD — pick NVIDIA if available, fall back to software ──────────────
if [[ -f /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
    export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
    echo "[carla_entrypoint] Using NVIDIA Vulkan ICD"
else
    echo "[carla_entrypoint] WARN: NVIDIA Vulkan ICD not found — GPU rendering may not work"
fi

# ── Launch ────────────────────────────────────────────────────────────────────
exec "$CARLA_BIN" \
    -RenderOffScreen \
    -nosound \
    -ros2 \
    -carla-primary-host="$CARLA_HOST" \
    -carla-primary-port="$CARLA_PORT" \
    $EXTRA_ARGS
