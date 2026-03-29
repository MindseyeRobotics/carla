# Building CARLA (UE5 + ROS2 + PX4 Bridge) in Docker

This guide covers building and running the **MindseyeRobotics fork** of CARLA inside Docker, including the `FlyingVehicles` plugin, native ROS2 support, and the PX4 HIL bridge.

> **Branch:** `implement-px4-closed-loop`  
> **Base:** `ue5-dev` (UE 5.5, Ubuntu 22.04)

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Docker ≥ 24 + Docker Compose v2 | `docker compose version` |
| NVIDIA GPU | RTX 3070+ recommended, 16 GB VRAM minimum |
| `nvidia-container-toolkit` | For GPU passthrough into Docker |
| ~300 GB free disk | UE5 source + build artifacts |
| GitHub account linked to Epic Games | Required to pull the UE5 fork |

### Install nvidia-container-toolkit (if not already done)

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 1. Clone the repo

```bash
git clone -b implement-px4-closed-loop \
  https://github.com/MindseyeRobotics/carla.git carla-merc
cd carla-merc
```

---

## 2. Create the build Dockerfile

Create `Dockerfile.build` in the repo root:

```dockerfile
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG GIT_LOCAL_CREDENTIALS

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    git-lfs \
    curl \
    wget \
    unzip \
    python3 \
    python3-pip \
    python3-dev \
    libssl-dev \
    libffi-dev \
    clang-14 \
    lld-14 \
    libc++-14-dev \
    libc++abi-14-dev \
    libomp-14-dev \
    libvulkan1 \
    libvulkan-dev \
    vulkan-tools \
    xdg-user-dirs \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Set clang-14 as default compiler (required by UE5)
RUN update-alternatives --install /usr/bin/clang   clang   /usr/bin/clang-14   100 \
 && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-14 100

# Python deps
RUN pip3 install --no-cache-dir setuptools wheel

WORKDIR /workspace/carla

# Copy source (use .dockerignore to exclude Build/ and large artifacts)
COPY . .

# Run CARLA setup — downloads UE5 fork and builds everything
# GIT_LOCAL_CREDENTIALS format: github_username@github_personal_access_token
RUN --mount=type=secret,id=git_creds \
    export GIT_LOCAL_CREDENTIALS=$(cat /run/secrets/git_creds) && \
    bash CarlaSetup.sh --skip-prerequisites
```

Create `.dockerignore` to keep the build context lean:

```
Build/
.git/
*.egg-info/
__pycache__/
*.pyc
dist/
```

---

## 3. Build the image

Store your GitHub credentials (username@token) in a file — never bake them into the image:

```bash
echo "your_github_username@your_personal_access_token" > .git_creds
chmod 600 .git_creds
```

Build (this will take **1–3 hours** on first run — UE5 is large):

```bash
DOCKER_BUILDKIT=1 docker build \
  --secret id=git_creds,src=.git_creds \
  -f Dockerfile.build \
  -t carla-merc:latest \
  .
```

> **Tip:** Use `--progress=plain` to see full build output. Add `--no-cache` to force a clean rebuild.

---

## 4. Runtime image (headless server)

For headless sim use (no display — what you want for training and HIL), create `Dockerfile.runtime`:

```dockerfile
FROM carla-merc:latest

# Expose CARLA ports
# 2000 — Python API (main RPC)
# 2001 — Traffic Manager
# 8080 — streaming (optional)
EXPOSE 2000 2001 8080

# Default: launch CARLA headless with ROS2 enabled
ENTRYPOINT ["/workspace/carla/Build/Package/CarlaUnreal.sh", \
            "-RenderOffScreen", \
            "-nosound", \
            "-ros2", \
            "-carla-primary-host=0.0.0.0", \
            "-carla-primary-port=2000"]
```

Build the runtime image:

```bash
docker build -f Dockerfile.runtime -t carla-merc-server:latest .
```

---

## 5. Run CARLA server

### Headless (training / HIL)

```bash
docker run --rm --gpus all \
  --network host \
  --name carla_server \
  carla-merc-server:latest
```

### With display (development / debugging)

```bash
xhost +local:docker

docker run --rm --gpus all \
  --network host \
  --name carla_server \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carla-merc-server:latest \
    -windowed -ResX=1280 -ResY=720
```

---

## 6. docker-compose stack (CARLA + PX4 SITL + HIL bridge)

The full MERC sim stack is managed from the **vertex repo** via `sim_start.sh`. But if you want to run CARLA + the HIL bridge standalone:

```yaml
# docker-compose.merc.yml
services:
  carla_server:
    image: carla-merc-server:latest
    container_name: carla_server
    network_mode: host
    runtime: nvidia
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    restart: unless-stopped

  carla_bridge:
    image: carla_bridge:latest   # built by vertex sim_start.sh --carla
    container_name: carla_bridge
    network_mode: host
    depends_on:
      - carla_server
    environment:
      - CARLA_HOST=localhost
      - CARLA_PORT=2000
      - MAVLINK_CONNECTION=udpin:0.0.0.0:14550
      - TAKEOFF_ALT=10.0
    restart: unless-stopped
```

```bash
docker compose -f docker-compose.merc.yml up
```

---

## 7. Verify the build

Once the server is running:

```bash
# Check CARLA is accepting connections
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
print('Connected — map:', world.get_map().name)
print('Blueprints:', len(list(world.get_blueprint_library())))
"

# Verify FlyingVehicles plugin is loaded (multirotor BPs available)
python3 -c "
import carla
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
lib = client.get_world().get_blueprint_library()
drones = [bp.id for bp in lib if 'multirotor' in bp.id.lower() or 'quadrotor' in bp.id.lower()]
print('Drone blueprints:', drones)
"
```

---

## 8. ROS2 + PX4 bridge

The HIL bridge script lives at `PythonAPI/examples/px4_carla_bridge.py`.

Run it directly (outside Docker, on host with CARLA visible):

```bash
python3 PythonAPI/examples/px4_carla_bridge.py \
  --host localhost \
  --port 2000 \
  --mavlink udpin:0.0.0.0:14550 \
  --takeoff-alt 10.0 \
  --verbose
```

Or via the vertex `sim_start.sh` (recommended):

```bash
# In the vertex repo:
./sim_start.sh -n 1 --carla
docker logs -f carla_bridge
```

---

## 9. CMake build flags reference

If rebuilding manually inside the container:

```bash
cmake -G Ninja -S . -B Build \
  --toolchain=$PWD/CMake/Toolchain.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ROS2=ON \
  -DBUILD_CARLA_UNREAL=ON \
  -DCARLA_UNREAL_ENGINE_PATH=$CARLA_UNREAL_ENGINE_PATH \
  -DLAUNCH_ARGS="-prefernvidia"

cmake --build Build
cmake --build Build --target carla-python-api-install
```

| Flag | Default | Notes |
|---|---|---|
| `ENABLE_ROS2` | OFF | Must be ON for PX4 bridge and ROS2 publishers |
| `BUILD_CARLA_UNREAL` | ON | Set OFF to build LibCarla only (no editor) |
| `CMAKE_BUILD_TYPE` | Release | Use `Debug` for symbol info |
| `CARLA_UNREAL_ENGINE_PATH` | — | Path to pre-built UE5 fork (saves ~1hr if cached) |

---

## 10. Disk space estimates

| Artifact | Size |
|---|---|
| UE5 source + build | ~225 GB |
| CARLA build output | ~40 GB |
| Docker image (build) | ~60 GB |
| Docker image (runtime, no UE5 src) | ~15 GB |

Use Docker BuildKit layer caching and a named volume for the UE5 path to avoid re-downloading between builds.

---

## Troubleshooting

**`vulkan: No DRI3 support`** — Add `--env NVIDIA_DRIVER_CAPABILITIES=all` and ensure `nvidia-container-toolkit` is configured.

**`Permission denied` on build artifacts** — CARLA cannot be built on external/NTFS-mounted disks. Use an ext4 volume.

**UE5 download fails / credential error** — Verify `GIT_LOCAL_CREDENTIALS` format is `username@token` (not `username:token`). Confirm your GitHub account is linked to Epic Games at [unrealengine.com/ue-on-github](https://www.unrealengine.com/en-US/ue-on-github).

**ROS2 publishers not working** — Ensure `-DENABLE_ROS2=ON` was passed to cmake and `./CarlaUnreal.sh --ros2` is in the launch args.

**FlyingVehicles BPs not appearing** — The `CarlaIntegrationPlugins` must be enabled in the UE5 project `.uproject` file. Check `Unreal/CarlaUnreal/CarlaUnreal.uproject` has `FlyingVehicles` listed as an enabled plugin.
