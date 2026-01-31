# Building and Running CARLA with IR Camera in Docker

This guide explains how to build and run CARLA with the re-enabled IR (Infrared/Thermal) camera plugin using Docker.

## Overview

The IR camera plugin is based on the FLIR Lepton 3.5 thermal camera specifications:
- **Resolution**: 160x120 pixels
- **Spectral Range**: 8-14 µm (long-wave infrared)
- **Frame Rate**: Up to 9 Hz (configurable)
- **Output**: Thermal data visualized as grayscale/colored depth-like imagery

## Prerequisites

- Docker installed (version 20.10 or later recommended)
- NVIDIA Docker runtime (nvidia-docker2) for GPU support
- At least 50GB of free disk space
- 16GB+ RAM recommended
- NVIDIA GPU with recent drivers (for rendering)

## Quick Start

### 1. Build the Docker Image

Create a Dockerfile in the CARLA root directory:

```dockerfile
FROM ubuntu:22.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    clang \
    lld \
    python3 \
    python3-pip \
    wget \
    software-properties-common \
    libpng-dev \
    libtiff5-dev \
    libjpeg-dev \
    tzdata \
    sed \
    curl \
    unzip \
    autoconf \
    libtool \
    rsync \
    libxml2-dev \
    git-lfs \
    && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /carla

# Copy CARLA source code
COPY . /carla/

# Set environment variables
ENV CARLA_ROOT=/carla
ENV CARLA_BUILD_TYPE=Release
ENV UE5_ROOT=/carla/UnrealEngine

# Note: Building CARLA requires Unreal Engine 5.5
# You must have access to the CARLA fork of UE5 from Epic Games
# This is a placeholder - actual UE5 setup requires authentication

# Build CARLA (this will take several hours)
# RUN cmake -G Ninja -S . -B Build \
#     --toolchain=$PWD/CMake/Toolchain.cmake \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DENABLE_ROS2=ON && \
#     cmake --build Build

# Expose ports
EXPOSE 2000-2002

# Set entrypoint
ENTRYPOINT ["/bin/bash"]
```

Build the image:
```bash
docker build -t carla-ir-camera:latest .
```

### 2. Run CARLA Server

```bash
docker run --name carla-server \
    --gpus all \
    -p 2000-2002:2000-2002 \
    -it carla-ir-camera:latest \
    /carla/Build/bin/CarlaUE5.sh -RenderOffScreen
```

### 3. Using the IR Camera in Python

Create a Python script to spawn and use the IR camera:

```python
import carla
import numpy as np
import cv2

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# Get blueprint library
blueprint_library = world.get_blueprint_library()

# Find IR camera blueprint
ir_camera_bp = blueprint_library.find('sensor.camera.ir')

# Set camera attributes (FLIR Lepton 3.5 specs)
ir_camera_bp.set_attribute('image_size_x', '160')
ir_camera_bp.set_attribute('image_size_y', '120')
ir_camera_bp.set_attribute('fov', '57')  # FLIR Lepton 3.5 FOV

# Spawn the camera
transform = carla.Transform(carla.Location(x=2.5, z=0.7))
ir_camera = world.spawn_actor(ir_camera_bp, transform)

# Callback to process IR camera data
def process_ir_image(image):
    # Convert to numpy array
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Remove alpha channel
    
    # Apply thermal colormap
    thermal_image = cv2.applyColorMap(array, cv2.COLORMAP_INFERNO)
    
    # Display or save
    cv2.imshow('IR Camera', thermal_image)
    cv2.waitKey(1)

# Register callback
ir_camera.listen(lambda image: process_ir_image(image))

# Keep script running
try:
    while True:
        world.wait_for_tick()
except KeyboardInterrupt:
    pass
finally:
    ir_camera.destroy()
    cv2.destroyAllWindows()
```

## Docker Compose Setup

For easier management, use Docker Compose. Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  carla-server:
    image: carla-ir-camera:latest
    container_name: carla-server
    runtime: nvidia
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    ports:
      - "2000-2002:2000-2002"
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./examples:/carla/examples
      - ./output:/carla/output
    command: /carla/Build/bin/CarlaUE5.sh -RenderOffScreen
    networks:
      - carla-network

  ros2-bridge:
    image: carla-ir-camera:latest
    container_name: carla-ros2-bridge
    depends_on:
      - carla-server
    environment:
      - ROS_DOMAIN_ID=0
    networks:
      - carla-network
    command: python3 /carla/PythonAPI/examples/ros2_bridge.py

networks:
  carla-network:
    driver: bridge
```

Start with:
```bash
docker-compose up
```

## ROS2 Integration

The IR camera publishes data to ROS2 topics:

### Topics Published:
- `/carla/[vehicle_id]/ir_camera/image` - IR camera image data (sensor_msgs/Image)
- `/carla/[vehicle_id]/ir_camera/camera_info` - Camera calibration info

### Subscribing to IR Camera Data:

```bash
# In the Docker container or on host with ROS2
ros2 topic list | grep ir_camera
ros2 topic echo /carla/ego_vehicle/ir_camera/image
ros2 topic hz /carla/ego_vehicle/ir_camera/image
```

### Visualizing with RViz2:

```bash
docker exec -it carla-server rviz2
```

Add image display and subscribe to `/carla/ego_vehicle/ir_camera/image`.

## Advanced Configuration

### Custom Thermal Shaders

To implement proper thermal physics simulation, you can create custom Unreal Engine materials:

1. Navigate to `Unreal/CarlaUnreal/Content/Carla/PostProcessingMaterials/`
2. Create a new material `ThermalEffectMaterial`
3. Implement thermal emission based on:
   - Object temperature properties
   - Material emissivity
   - Environmental factors
4. Update `IRCamera.cpp` to use the new material

### Performance Tuning

Modify sensor tick rate for performance:

```python
ir_camera_bp.set_attribute('sensor_tick', '0.111')  # ~9 Hz (FLIR Lepton 3.5)
```

### Multi-Camera Setup

Spawn multiple IR cameras for different viewpoints:

```python
# Front camera
front_transform = carla.Transform(
    carla.Location(x=2.5, z=0.7),
    carla.Rotation(pitch=0, yaw=0)
)
ir_camera_front = world.spawn_actor(ir_camera_bp, front_transform, attach_to=vehicle)

# Side camera
side_transform = carla.Transform(
    carla.Location(x=0.0, y=1.5, z=0.7),
    carla.Rotation(pitch=0, yaw=90)
)
ir_camera_side = world.spawn_actor(ir_camera_bp, side_transform, attach_to=vehicle)
```

## Troubleshooting

### GPU Not Detected
```bash
# Verify NVIDIA Docker runtime
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### Port Already in Use
```bash
# Check what's using port 2000
sudo lsof -i :2000
# Kill the process or use different port mapping
docker run -p 2010:2000 ...
```

### IR Camera Not Available
Verify the sensor is registered:
```python
import carla
client = carla.Client('localhost', 2000)
world = client.get_world()
blueprints = world.get_blueprint_library()
ir_cameras = blueprints.filter('sensor.camera.ir')
print(f"Found {len(ir_cameras)} IR camera blueprint(s)")
```

### Build Errors

If you encounter build errors:
1. Ensure you have access to Unreal Engine 5.5 (requires Epic Games account linked to GitHub)
2. Check disk space: `df -h`
3. Verify CMake version: `cmake --version` (should be 3.27.2+)
4. Check build logs in `Build/` directory

## Resources

- [CARLA Documentation](https://carla.readthedocs.io/)
- [FLIR Lepton 3.5 Datasheet](https://www.flir.com/products/lepton/)
- [ROS2 Bridge Documentation](https://carla.readthedocs.io/en/latest/ros2_bridge/)
- [Docker Documentation](https://docs.docker.com/)

## Support

For issues related to:
- IR Camera Plugin: Open an issue on this repository
- CARLA Core: [CARLA GitHub Issues](https://github.com/carla-simulator/carla/issues)
- Docker: [Docker Community](https://forums.docker.com/)

## License

This work is licensed under the terms of the MIT license. See LICENSE file for details.
