# ROS2 PX4 Bridge for CARLA

## Overview

The ROS2PX4Bridge provides a bidirectional communication bridge between CARLA simulator and PX4 autopilot systems. It enables seamless integration of PX4-based autonomous vehicles within CARLA's simulation environment.

## Architecture

### Components

1. **ROS2PX4Bridge.h/cpp** - Main bridge implementation
   - Location: `LibCarla/source/carla/ros2/ROS2PX4Bridge.h/cpp`
   - Manages communication between CARLA and PX4
   - Handles coordinate system transformations

2. **PX4ActuatorControls** - Message type for receiving PX4 control commands
   - Location: `LibCarla/source/carla/ros2/types/PX4ActuatorControls.h/cpp`
   - Includes PubSubTypes for Fast-DDS serialization

3. **Integration with ROS2 Singleton** - Lifecycle management
   - Modified: `LibCarla/source/carla/ros2/ROS2.h/cpp`
   - Added PX4 bridge member and control methods

## Features

### Publishers (CARLA → PX4)

The bridge publishes the following data from CARLA to PX4:

1. **Vehicle Odometry** (`/fmu/in/vehicle_odometry`)
   - Position and orientation
   - Linear and angular velocities
   - Uses standard `nav_msgs/msg/Odometry`

2. **IMU Data** (`/fmu/in/vehicle_imu`)
   - Linear acceleration
   - Angular velocity
   - Orientation (from compass)
   - Uses standard `sensor_msgs/msg/Imu`

3. **GPS Position** (`/fmu/in/vehicle_gps_position`)
   - Latitude, longitude, altitude
   - Uses standard `sensor_msgs/msg/NavSatFix`

### Subscribers (PX4 → CARLA)

1. **Actuator Controls** (`/fmu/out/actuator_controls_0`)
   - Roll, pitch, yaw, throttle commands
   - Uses custom `px4_msgs/msg/PX4ActuatorControls`

## Coordinate System Transformations

### CARLA Coordinate System (UE4)
- X: Forward
- Y: Right
- Z: Up
- Right-handed coordinate system

### ROS/PX4 Coordinate System
- X: Forward
- Y: Left
- Z: Up
- Right-handed coordinate system

### Transformation Applied
The bridge automatically converts between these coordinate systems:
- Y-axis is negated (right ↔ left)
- Quaternion orientations are adjusted accordingly

## Usage

### Enabling the PX4 Bridge

```cpp
#include "carla/ros2/ROS2.h"

// Get ROS2 singleton
auto ros2 = carla::ros2::ROS2::GetInstance();

// Enable PX4 bridge for a vehicle
void* vehicle_ptr = /* pointer to vehicle actor */;
std::string vehicle_name = "ego_vehicle";
ros2->EnablePX4Bridge(vehicle_ptr, vehicle_name);
```

### Updating Vehicle State

```cpp
// In your vehicle update loop
carla::geom::Transform transform = vehicle->GetTransform();
carla::geom::Vector3D velocity = vehicle->GetVelocity();
carla::geom::Vector3D angular_velocity = vehicle->GetAngularVelocity();
carla::geom::Vector3D accelerometer = imu->GetAccelerometer();
carla::geom::Vector3D gyroscope = imu->GetGyroscope();
float compass = imu->GetCompass();
carla::geom::GeoLocation gps_location = gps->GetLocation();

ros2->UpdatePX4Bridge(
    transform,
    velocity,
    angular_velocity,
    accelerometer,
    gyroscope,
    compass,
    gps_location
);
```

### Disabling the PX4 Bridge

```cpp
ros2->DisablePX4Bridge();
```

### Checking Bridge Status

```cpp
if (ros2->IsPX4BridgeEnabled()) {
    // Bridge is active
}
```

## PX4 Topic Mapping

| Data Type | CARLA → PX4 Topic | Message Type |
|-----------|-------------------|--------------|
| Odometry | `/fmu/in/vehicle_odometry` | `nav_msgs::msg::Odometry` |
| IMU | `/fmu/in/vehicle_imu` | `sensor_msgs::msg::Imu` |
| GPS | `/fmu/in/vehicle_gps_position` | `sensor_msgs::msg::NavSatFix` |

| Data Type | PX4 → CARLA Topic | Message Type |
|-----------|-------------------|--------------|
| Actuator Controls | `/fmu/out/actuator_controls_0` | `px4_msgs::msg::PX4ActuatorControls` |

## PX4 Actuator Controls Mapping

The `PX4ActuatorControls` message contains an array of 8 float values:

- `control[0]`: Roll control
- `control[1]`: Pitch control
- `control[2]`: Yaw control
- `control[3]`: Throttle control
- `control[4-7]`: Reserved for additional controls

Values are normalized to the range [-1.0, 1.0].

## Integration with PX4

To use this bridge with PX4:

1. **Start CARLA with ROS2 enabled**
   ```bash
   ./CarlaUE4.sh -ros2
   ```

2. **Launch PX4 SITL**
   ```bash
   make px4_sitl_rtps gazebo
   ```

3. **Configure PX4 to use CARLA odometry**
   - Set `EKF2_AID_MASK` to enable external vision
   - Set appropriate EKF2 parameters for vision-based navigation

4. **Enable the bridge in your CARLA script**
   ```python
   import carla
   
   client = carla.Client('localhost', 2000)
   world = client.get_world()
   
   # Enable ROS2
   settings = world.get_settings()
   settings.enable_ros2 = True
   world.apply_settings(settings)
   
   # The bridge can be enabled via the C++ API
   ```

## Thread Safety

- The bridge uses Fast-DDS which is thread-safe for publishing and subscribing
- Each bridge instance maintains its own DDS participant
- The implementation uses shared_ptr for safe resource management

## Error Handling

The bridge includes error handling for:
- Failed DDS participant creation
- Topic creation failures
- Publisher/Subscriber creation failures
- Message serialization errors

All errors are logged to stderr with descriptive messages.

## Performance Considerations

- Messages are published at the CARLA simulation frequency
- Fast-DDS uses efficient zero-copy mechanisms where possible
- Coordinate transformations are lightweight (matrix operations)

## Future Enhancements

Potential areas for expansion:

1. **Additional PX4 Messages**
   - Vehicle commands
   - Offboard control mode
   - Vehicle status

2. **Bidirectional Vehicle Control**
   - Full integration of actuator controls with CARLA vehicle physics
   - Support for different vehicle types (multirotor, fixed-wing, rover)

3. **Advanced Coordinate Transformations**
   - Support for FRD (Front-Right-Down) convention
   - NED (North-East-Down) frame support

4. **Quality of Service (QoS) Configuration**
   - Configurable reliability and durability settings
   - Latency optimization

## Dependencies

- Fast-DDS (eProsima)
- CARLA LibCarla
- C++14 or later

## License

Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).

This work is licensed under the terms of the MIT license.
