# PX4 Integration for CARLA

This document provides comprehensive instructions for integrating PX4 autopilot with CARLA's aerial drone simulation via ROS2.

## Overview

This integration allows you to:
- Run PX4 SITL (Software-In-The-Loop) with CARLA as the physics simulator
- Test autonomous flight algorithms in realistic urban environments
- Develop and validate mission planning with real-world scenarios
- Use all CARLA sensors (cameras, LiDAR, IR thermal camera, etc.) with PX4
- Execute MAVLink commands and offboard control

## Architecture

```
┌─────────────┐      MAVLink/UDP      ┌──────────────┐
│   PX4 SITL  │ ◄──────────────────► │  MAVROS-ROS2 │
│  Autopilot  │                       │    Bridge    │
└─────────────┘                       └──────┬───────┘
                                             │ ROS2
                                             │ Topics
                                      ┌──────▼───────┐
                                      │ CARLA-PX4    │
                                      │ ROS2 Bridge  │
                                      └──────┬───────┘
                                             │
                                      ┌──────▼───────┐
                                      │    CARLA     │
                                      │  Multirotor  │
                                      │   Physics    │
                                      └──────────────┘
```

## Prerequisites

### Software Requirements

- **CARLA** with IR camera plugin (this repository)
- **PX4-Autopilot** (v1.14 or later)
- **ROS2** Humble or later
- **MAVROS** ROS2 package
- **QGroundControl** (optional, for monitoring)
- **Docker** (optional, for containerized setup)

### System Requirements

- Ubuntu 22.04 LTS (recommended)
- 16GB+ RAM
- NVIDIA GPU (for CARLA rendering)
- ~20GB free disk space

## Installation

### 1. Install PX4-Autopilot

```bash
# Clone PX4
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install dependencies
bash ./Tools/setup/ubuntu.sh

# Build for SITL
make px4_sitl_default
```

### 2. Install MAVROS for ROS2

```bash
# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

### 3. Build CARLA with PX4 Integration

```bash
cd /path/to/carla

# Configure with ROS2 enabled
cmake -G Ninja -S . -B Build \
    --toolchain=$PWD/CMake/Toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_ROS2=ON

# Build
cmake --build Build
```

## Configuration

### PX4 Configuration for CARLA

Create a custom PX4 startup script for CARLA integration:

```bash
# In PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/
# Create file: 4019_carla_multirotor
```

```bash
#!/bin/sh
# @name CARLA Multirotor
# @type Quadrotor

. ${R}etc/init.d/rc.mc_defaults

param set-default MAV_TYPE 2
param set-default SYS_AUTOSTART 4019

# Disable some checks for SITL
param set-default COM_RC_IN_MODE 1
param set-default NAV_RCL_ACT 0
param set-default NAV_DLL_ACT 0

# Multirotor mixer
set MIXER quad_x
set PWM_OUT 1234
```

### CARLA PX4 Bridge Configuration

Create `config/px4_bridge_config.yaml`:

```yaml
# PX4-CARLA Bridge Configuration

mavlink:
  # UDP ports for MAVLink communication
  local_port: 14540
  remote_port: 14557
  remote_address: "127.0.0.1"
  system_id: 1
  component_id: 1

drone:
  # Default spawn location (Unreal coordinates)
  spawn_location:
    x: 0.0
    y: 0.0
    z: 50.0  # Spawn 50m above ground
  
  # FLIR Lepton 3.5 specifications
    resolution: [160, 120]
    fov: 57
    update_rate: 9
    
  sensors:
    imu:
      enabled: true
      update_rate: 100  # Hz
      topic: "/carla/drone/imu"
    
    gps:
      enabled: true
      update_rate: 10   # Hz
      topic: "/carla/drone/gps"
    
    barometer:
      enabled: true
      update_rate: 50   # Hz
      topic: "/carla/drone/baro"
    
    camera:
      enabled: true
      type: "rgb"
      resolution: [640, 480]
      fov: 90
      update_rate: 30   # Hz
    
    ir_camera:
      enabled: true
      resolution: [160, 120]
      fov: 57  # FLIR Lepton 3.5
      update_rate: 9   # Hz

physics:
  # Multirotor parameters
  mass: 1.5  # kg
  arm_length: 0.25  # m
  rotor_thrust_coeff: 8.54858e-06
  rotor_torque_coeff: 0.016
  max_rotor_speed: 1000  # rad/s
```

## Usage

### Launch CARLA Server

```bash
# Terminal 1: Start CARLA
cd /path/to/carla
./Build/bin/CarlaUE5.sh -RenderOffScreen
```

### Launch PX4 SITL

```bash
# Terminal 2: Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl none_iris

# Or with custom model:
PX4_SYS_AUTOSTART=4019 make px4_sitl none
```

### Launch MAVROS Bridge

```bash
# Terminal 3: Start MAVROS
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

### Launch CARLA-PX4 Bridge

```bash
# Terminal 4: Start CARLA-PX4 Bridge
cd /path/to/carla
ros2 launch carla_px4_bridge px4_bridge.launch.py
```

### Spawn Drone in CARLA

```python
# Python script or via ROS2 service
import carla

client = carla.Client('localhost', 2000)
world = client.get_world()

# Get multirotor blueprint
blueprint_library = world.get_blueprint_library()
drone_bp = blueprint_library.find('vehicle.multirotor.px4')

# Spawn drone
transform = carla.Transform(carla.Location(x=0, y=0, z=50))
drone = world.spawn_actor(drone_bp, transform)

print(f"Drone spawned with ID: {drone.id}")
```

## ROS2 Topics

### Published by CARLA-PX4 Bridge

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/carla/drone/imu` | `sensor_msgs/Imu` | 100 Hz | IMU data (accelerometer, gyroscope) |
| `/carla/drone/gps` | `sensor_msgs/NavSatFix` | 10 Hz | GPS position |
| `/carla/drone/baro` | `sensor_msgs/FluidPressure` | 50 Hz | Barometric pressure |
| `/carla/drone/camera/image` | `sensor_msgs/Image` | 30 Hz | RGB camera |
| `/carla/drone/ir_camera/image` | `sensor_msgs/Image` | 9 Hz | Thermal IR camera (FLIR Lepton 3.5) |
| `/carla/drone/lidar` | `sensor_msgs/PointCloud2` | 10 Hz | LiDAR point cloud (optional) |
| `/carla/drone/odometry` | `nav_msgs/Odometry` | 100 Hz | Ground truth odometry |

### Subscribed by CARLA-PX4 Bridge

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/mavros/actuator_control` | `mavros_msgs/ActuatorControl` | Motor commands from PX4 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Attitude setpoint |

### MAVROS Topics (PX4 Interface)

Standard MAVROS topics are available:
- `/mavros/state` - Flight controller state
- `/mavros/local_position/pose` - Local position
- `/mavros/setpoint_position/local` - Position setpoint
- `/mavros/cmd/arming` - Arm/disarm service
- `/mavros/set_mode` - Flight mode service

## Flight Modes

### Supported Modes

- **Manual**: Direct control via RC or joystick
- **Stabilize**: Attitude stabilization
- **Altitude Hold**: Maintain altitude
- **Position Hold**: Hold GPS position
- **Offboard**: External computer control (ROS2)
- **Mission**: Automated waypoint following
- **Return to Land (RTL)**: Automatic return and landing

### Example: Offboard Control

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
        # Services
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for publishing setpoints
        self.timer = self.create_timer(0.05, self.publish_setpoint)  # 20 Hz
        
        self.current_state = State()
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 10.0  # 10m altitude
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def publish_setpoint(self):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target_pose)
    
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        return future
    
    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        return future

def main():
    rclpy.init()
    node = OffboardControl()
    
    # Wait for PX4 connection
    rate = node.create_rate(20)
    while not node.current_state.connected:
        rclpy.spin_once(node)
        rate.sleep()
    
    # Send a few setpoints before starting
    for _ in range(100):
        node.publish_setpoint()
        rclpy.spin_once(node)
        rate.sleep()
    
    # Set offboard mode and arm
    node.set_offboard_mode()
    node.arm()
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Mission Planning with QGroundControl

### Connect QGroundControl

1. Launch QGroundControl
2. Go to **Application Settings** → **Comm Links**
3. Add new link:
   - Type: UDP
   - Port: 14550
4. Connect to PX4 SITL

### Plan a Mission

1. Click **Plan** tab
2. Click on map to add waypoints
3. Set altitude and speed for each waypoint
4. Upload mission to PX4
5. Switch to **Mission** mode in **Fly** tab
6. Arm and takeoff

The drone will execute the mission in CARLA's environment.

## Docker Setup

### Docker Compose

Create `docker-compose-px4.yml`:

```yaml
version: '3.8'

services:
  carla-server:
    image: carlasim/carla:0.9.15
    runtime: nvidia
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
    ports:
      - "2000-2002:2000-2002"
    command: /bin/bash CarlaUE4.sh -RenderOffScreen
    networks:
      - px4-network

  px4-sitl:
    image: px4io/px4-dev-simulation-focal:latest
    environment:
      - PX4_SYS_AUTOSTART=4019
    ports:
      - "14540:14540/udp"
      - "14557:14557/udp"
    volumes:
      - ./PX4-Autopilot:/px4
    command: make px4_sitl none
    networks:
      - px4-network

  mavros:
    image: ros:humble
    environment:
      - ROS_DOMAIN_ID=0
    ports:
      - "14550:14550/udp"
    command: ros2 launch mavros px4.launch fcu_url:=udp://:14540@px4-sitl:14557
    depends_on:
      - px4-sitl
    networks:
      - px4-network

  carla-px4-bridge:
    image: carla-px4-bridge:latest
    environment:
      - ROS_DOMAIN_ID=0
    depends_on:
      - carla-server
      - mavros
    command: ros2 launch carla_px4_bridge px4_bridge.launch.py
    networks:
      - px4-network

networks:
  px4-network:
    driver: bridge
```

Launch with:
```bash
docker-compose -f docker-compose-px4.yml up
```

## Troubleshooting

### PX4 Not Connecting to MAVROS

**Check UDP ports:**
```bash
# Verify PX4 is listening
netstat -an | grep 14540

# Check MAVROS connection
ros2 topic echo /mavros/state
```

**Solution**: Ensure firewall allows UDP ports 14540, 14550, 14557

### Drone Not Spawning in CARLA

**Check blueprint:**
```python
blueprints = world.get_blueprint_library()
multirotors = blueprints.filter('vehicle.multirotor*')
for bp in multirotors:
    print(bp.id)
```

**Solution**: Verify FlyingVehicles plugin is loaded

### IMU/GPS Data Not Publishing

**Check ROS2 topics:**
```bash
ros2 topic list | grep carla
ros2 topic hz /carla/drone/imu
```

**Solution**: Verify CARLA-PX4 bridge is running and sensors are enabled in config

### Poor Flight Performance

**Tune PX4 parameters:**
```bash
# In PX4 console (px4-sitl shell)
param set MC_ROLLRATE_P 0.15
param set MC_PITCHRATE_P 0.15
param save
```

### IR Camera Not Working

**Verify IR camera is available:**
```bash
ros2 topic list | grep ir_camera
ros2 topic echo /carla/drone/ir_camera/image --once
```

**Solution**: Ensure IR camera plugin is enabled in CARLA build

## Advanced Topics

### Custom Drone Models

Create custom drone configurations in `config/drone_models/`:

```yaml
# custom_hexacopter.yaml
name: "CustomHexacopter"
type: "hexacopter"
mass: 2.5
arms: 6
arm_length: 0.3
rotors:
  - position: [0.3, 0.0, 0.0]
    direction: 1
  - position: [-0.15, 0.26, 0.0]
    direction: -1
  # ... more rotors
```

### Multi-Drone Simulation

```python
# Spawn multiple drones
for i in range(5):
    transform = carla.Transform(
        carla.Location(x=i*5, y=0, z=50)
    )
    drone = world.spawn_actor(drone_bp, transform)
    print(f"Drone {i} spawned: {drone.id}")
```

### Integration with ROS2 Navigation Stack

Use Nav2 for autonomous navigation:
```bash
ros2 launch nav2_bringup navigation_launch.py
```

## Performance Tuning

### Optimize for Real-Time

```yaml
# In px4_bridge_config.yaml
performance:
  use_sim_time: true
  sensor_thread_priority: 99
  actuator_thread_priority: 95
  max_update_rate: 1000  # Hz
```

### GPU Acceleration

Ensure CARLA uses GPU:
```bash
nvidia-smi  # Verify GPU is detected
export CUDA_VISIBLE_DEVICES=0
```

## References

- [PX4 User Guide](https://docs.px4.io/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [QGroundControl User Guide](https://docs.qgroundcontrol.com/)
- [CARLA Documentation](https://carla.readthedocs.io/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

- PX4 Integration Issues: Open issue on this repository
- PX4 Autopilot: [PX4 Discuss](https://discuss.px4.io/)
- CARLA: [CARLA GitHub](https://github.com/carla-simulator/carla/issues)
- MAVROS: [MAVROS GitHub](https://github.com/mavlink/mavros)

## License

MIT License - See LICENSE file for details
