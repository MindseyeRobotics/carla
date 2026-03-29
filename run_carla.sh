#!/usr/bin/env bash
# run_carla.sh — build and/or run the CARLA UE5 Docker container
#
# Usage:
#   ./run_carla.sh                  # run headless (image must exist)
#   ./run_carla.sh --build          # build image then run
#   ./run_carla.sh --build-only     # build image, don't run
#   ./run_carla.sh --display        # run with GUI window (requires X11)
#   ./run_carla.sh --down           # stop and remove the container
#   ./run_carla.sh --verify         # check CARLA is accepting connections
#
# Environment variables:
#   GIT_CREDS_FILE   Path to file containing "github_user@github_token"
#                    (default: .git_creds in repo root)
#   CARLA_PORT       Port for Python API (default: 2000)
#   BUILD_IMAGE      Image name for build stage (default: carla-merc)
#   RUNTIME_IMAGE    Image name for runtime stage (default: carla-merc-server)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GIT_CREDS_FILE="${GIT_CREDS_FILE:-$REPO_ROOT/.git_creds}"
CARLA_PORT="${CARLA_PORT:-2000}"
BUILD_IMAGE="${BUILD_IMAGE:-carla-merc}"
RUNTIME_IMAGE="${RUNTIME_IMAGE:-carla-merc-server}"
CONTAINER_NAME="carla_server"

# ── Colour helpers ────────────────────────────────────────────────────────────
log()  { echo -e "\033[1;34m[carla]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[carla]\033[0m $*"; }
err()  { echo -e "\033[1;31m[carla]\033[0m $*" >&2; }
warn() { echo -e "\033[1;33m[carla]\033[0m $*"; }

# ── Arg parse ─────────────────────────────────────────────────────────────────
DO_BUILD=false
BUILD_ONLY=false
WITH_DISPLAY=false
DO_DOWN=false
DO_VERIFY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --build)      DO_BUILD=true;  shift ;;
        --build-only) DO_BUILD=true; BUILD_ONLY=true; shift ;;
        --display)    WITH_DISPLAY=true; shift ;;
        --down)       DO_DOWN=true;   shift ;;
        --verify)     DO_VERIFY=true; shift ;;
        *)
            err "Unknown option: $1"
            echo "Usage: $0 [--build] [--build-only] [--display] [--down] [--verify]"
            exit 1
            ;;
    esac
done

# ── Teardown ──────────────────────────────────────────────────────────────────
if [[ "$DO_DOWN" == "true" ]]; then
    log "Stopping $CONTAINER_NAME ..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm   "$CONTAINER_NAME" 2>/dev/null || true
    ok "Done."
    exit 0
fi

# ── Verify ────────────────────────────────────────────────────────────────────
if [[ "$DO_VERIFY" == "true" ]]; then
    log "Verifying CARLA on localhost:$CARLA_PORT ..."
    python3 - <<PYEOF
import sys
try:
    import carla
    client = carla.Client('localhost', int('$CARLA_PORT'))
    client.set_timeout(10.0)
    world = client.get_world()
    lib   = world.get_blueprint_library()
    drones = [bp.id for bp in lib if any(k in bp.id.lower() for k in ('multirotor','quadrotor','flying'))]
    print(f"  Connected    : localhost:$CARLA_PORT")
    print(f"  Map          : {world.get_map().name}")
    print(f"  Blueprints   : {len(list(lib))}")
    print(f"  Drone BPs    : {drones if drones else '(none — FlyingVehicles plugin may not be loaded)'}")
    sys.exit(0)
except Exception as e:
    print(f"  FAILED: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF
    exit $?
fi

# ── Build ─────────────────────────────────────────────────────────────────────
if [[ "$DO_BUILD" == "true" ]]; then
    # Credentials check
    if [[ ! -f "$GIT_CREDS_FILE" ]]; then
        err "Git credentials file not found: $GIT_CREDS_FILE"
        err "Create it with: echo 'github_username@your_token' > .git_creds && chmod 600 .git_creds"
        err "Your GitHub account must be linked to Epic Games to pull the UE5 fork."
        exit 1
    fi

    log "Building build image ($BUILD_IMAGE) — this will take 1-3 hours on first run ..."
    warn "Disk requirement: ~300 GB free"

    DOCKER_BUILDKIT=1 docker build \
        --secret id=git_creds,src="$GIT_CREDS_FILE" \
        -f "$REPO_ROOT/Dockerfile.build" \
        -t "$BUILD_IMAGE:latest" \
        "$REPO_ROOT"

    ok "Build image ready: $BUILD_IMAGE:latest"

    log "Building runtime image ($RUNTIME_IMAGE) ..."
    docker build \
        -f "$REPO_ROOT/Dockerfile.runtime" \
        -t "$RUNTIME_IMAGE:latest" \
        "$REPO_ROOT"

    ok "Runtime image ready: $RUNTIME_IMAGE:latest"

    [[ "$BUILD_ONLY" == "true" ]] && exit 0
fi

# ── Check runtime image exists ────────────────────────────────────────────────
if ! docker image inspect "$RUNTIME_IMAGE:latest" &>/dev/null; then
    err "Runtime image '$RUNTIME_IMAGE:latest' not found."
    err "Run with --build first: $0 --build"
    exit 1
fi

# ── Stop any existing container ───────────────────────────────────────────────
if docker inspect "$CONTAINER_NAME" &>/dev/null; then
    log "Removing existing container ..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm   "$CONTAINER_NAME" 2>/dev/null || true
fi

# ── Build docker run args ─────────────────────────────────────────────────────
RUN_ARGS=(
    --rm
    --gpus all
    --network host
    --name "$CONTAINER_NAME"
    --env CARLA_PORT="$CARLA_PORT"
    --env NVIDIA_VISIBLE_DEVICES=all
    --env NVIDIA_DRIVER_CAPABILITIES=all
    --env __GLX_VENDOR_LIBRARY_NAME=nvidia
    --env __NV_PRIME_RENDER_OFFLOAD=1
    --env SDL_VIDEODRIVER=offscreen
)

if [[ "$WITH_DISPLAY" == "true" ]]; then
    if [[ -z "${DISPLAY:-}" ]]; then
        warn "DISPLAY not set — defaulting to :0"
        export DISPLAY=:0
    fi
    log "Granting X11 access to Docker ..."
    xhost +local:docker 2>/dev/null || warn "xhost not available — window may not appear"

    RUN_ARGS+=(
        --env DISPLAY="$DISPLAY"
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
        --env SDL_VIDEODRIVER=x11
    )

    # Override entrypoint flags for windowed mode
    WINDOW_ARGS="-windowed -ResX=1280 -ResY=720"
    RUN_ARGS+=(--env CARLA_EXTRA_ARGS="$WINDOW_ARGS")

    log "Launching CARLA with display ($DISPLAY) ..."
else
    log "Launching CARLA headless on port $CARLA_PORT ..."
fi

docker run "${RUN_ARGS[@]}" "$RUNTIME_IMAGE:latest"
