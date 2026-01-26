# IR Camera Sensor - FLIR Lepton 3.5

This document describes the IR (Infrared/Thermal) camera sensor implementation in CARLA, based on the FLIR Lepton 3.5 thermal camera specifications.

## Overview

The IR camera sensor simulates a thermal imaging camera that captures infrared radiation in the 8-14 µm wavelength range. This sensor can be used for autonomous driving scenarios requiring thermal imaging capabilities, such as pedestrian detection in low-light conditions, heat signature tracking, or nighttime navigation.

## Specifications

Based on FLIR Lepton 3.5:

| Parameter | Value |
|-----------|-------|
| Resolution | 160x120 pixels (configurable) |
| Spectral Range | 8-14 µm (long-wave infrared) |
| Frame Rate | Up to 9 Hz (configurable) |
| Field of View | 57° (FLIR Lepton 3.5 default) |
| Output Format | BGRA8 (sensor_msgs/Image) |

## Usage

### Python API

```python
import carla

# Connect to CARLA
client = carla.Client('localhost', 2000)
world = client.get_world()

# Get IR camera blueprint
blueprint_library = world.get_blueprint_library()
ir_camera_bp = blueprint_library.find('sensor.camera.ir')

# Configure sensor attributes (FLIR Lepton 3.5 defaults)
ir_camera_bp.set_attribute('image_size_x', '160')
ir_camera_bp.set_attribute('image_size_y', '120')
ir_camera_bp.set_attribute('fov', '57')
ir_camera_bp.set_attribute('sensor_tick', '0.111')  # ~9 Hz

# Spawn the camera
transform = carla.Transform(carla.Location(x=2.5, z=0.7))
ir_camera = world.spawn_actor(ir_camera_bp, transform)

# Register callback
def process_image(image):
    # Process thermal image
    print(f"IR image: {image.width}x{image.height}")

ir_camera.listen(process_image)
```

### Attaching to Vehicle

```python
# Get a vehicle
vehicle = world.spawn_actor(
    blueprint_library.find('vehicle.tesla.model3'),
    random.choice(world.get_map().get_spawn_points())
)

# Attach IR camera to vehicle
ir_transform = carla.Transform(carla.Location(x=2.5, z=0.7))
ir_camera = world.spawn_actor(ir_camera_bp, ir_transform, attach_to=vehicle)
```

### Processing Thermal Data

```python
import numpy as np
import cv2

def visualize_thermal(image):
    # Convert to numpy array
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]  # Remove alpha channel
    
    # Apply thermal colormap for visualization
    thermal_colored = cv2.applyColorMap(array, cv2.COLORMAP_INFERNO)
    
    # Display or save
    cv2.imshow('Thermal Camera', thermal_colored)
    cv2.waitKey(1)

ir_camera.listen(visualize_thermal)
```

## ROS2 Topics

When the ROS2 bridge is enabled, the IR camera publishes data to the following topics:

### Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/carla/{vehicle_id}/ir_camera/image` | `sensor_msgs/Image` | Raw thermal image data in BGRA8 format |
| `/carla/{vehicle_id}/ir_camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration and metadata |

### Topic Details

**Image Topic** (`/carla/{vehicle_id}/ir_camera/image`)
- **Encoding**: `bgra8`
- **Dimensions**: 160x120 (default, configurable)
- **Frame ID**: `ir_camera`
- **Publishing Rate**: Up to 9 Hz (configurable via `sensor_tick`)

**Camera Info Topic** (`/carla/{vehicle_id}/ir_camera/camera_info`)
- **Header**: Synchronized with image timestamp
- **Calibration**: Includes intrinsic camera matrix (K), distortion coefficients (D), and projection matrix (P)
- **ROI**: Region of interest (configurable)

### Subscribing to Topics

```bash
# List available IR camera topics
ros2 topic list | grep ir_camera

# Echo image data
ros2 topic echo /carla/ego_vehicle/ir_camera/image

# Check publishing rate
ros2 topic hz /carla/ego_vehicle/ir_camera/image

# View camera info
ros2 topic echo /carla/ego_vehicle/ir_camera/camera_info
```

### Visualizing with RViz2

```bash
# Launch RViz2
rviz2

# Add -> By topic -> /carla/ego_vehicle/ir_camera/image -> Image
# Set Fixed Frame to "map" or vehicle frame
```

## Configurable Attributes

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_size_x` | int | 160 | Image width in pixels |
| `image_size_y` | int | 120 | Image height in pixels |
| `fov` | float | 90.0 | Horizontal field of view in degrees |
| `sensor_tick` | float | 0.0 | Simulation seconds between sensor captures (0.0 = every tick) |

### Example Configuration

```python
# High resolution (custom)
ir_camera_bp.set_attribute('image_size_x', '320')
ir_camera_bp.set_attribute('image_size_y', '240')

# Match FLIR Lepton 3.5 FOV
ir_camera_bp.set_attribute('fov', '57')

# 9 Hz update rate (FLIR Lepton 3.5 max)
ir_camera_bp.set_attribute('sensor_tick', '0.111')
```

## Limitations

### Current Implementation

1. **Thermal Physics Simulation**
   - The current implementation uses depth-based rendering as a proxy for thermal visualization
   - Does not simulate true thermal physics (object temperature, emissivity, heat transfer)
   - Depth correlation provides reasonable approximation for object detection scenarios

2. **Environmental Effects**
   - Does not simulate atmospheric attenuation or scattering
   - No weather effects on thermal imaging (fog, rain, etc.)
   - Constant emissivity assumed for all materials

3. **Performance**
   - Higher resolutions or faster frame rates may impact simulation performance
   - Recommended to use FLIR Lepton 3.5 defaults (160x120 @ 9 Hz) for optimal performance

4. **Visualization**
   - Thermal data is rendered using depth shader approximation
   - True thermal rendering would require custom shaders with temperature-based emission

### Future Enhancements

To achieve more realistic thermal imaging, future versions could include:

- **Thermal Material Properties**: Object temperature and emissivity values
- **Environmental Simulation**: Atmospheric effects, weather impact
- **Custom Thermal Shaders**: GPU-accelerated thermal physics rendering
- **Calibrated Output**: Temperature-mapped pixel values instead of depth proxy
- **Dynamic Range**: Support for high dynamic range thermal imaging

## Best Practices

### Performance Optimization

```python
# Use lower resolution for better performance
ir_camera_bp.set_attribute('image_size_x', '80')
ir_camera_bp.set_attribute('image_size_y', '60')

# Reduce update rate if real-time is not critical
ir_camera_bp.set_attribute('sensor_tick', '0.2')  # 5 Hz
```

### Multi-Sensor Setup

```python
# Front thermal camera
front_ir = world.spawn_actor(
    ir_camera_bp,
    carla.Transform(carla.Location(x=2.5, z=0.7)),
    attach_to=vehicle
)

# Rear thermal camera
rear_ir = world.spawn_actor(
    ir_camera_bp,
    carla.Transform(
        carla.Location(x=-2.5, z=0.7),
        carla.Rotation(yaw=180)
    ),
    attach_to=vehicle
)
```

### Data Recording

```python
# Save thermal images to disk
def save_thermal_image(image):
    image.save_to_disk(f'_out/ir_{image.frame:06d}.png')

ir_camera.listen(save_thermal_image)
```

## Troubleshooting

### Sensor Not Found

If the IR camera blueprint is not available:

```python
# Verify sensor is registered
ir_cameras = blueprint_library.filter('sensor.camera.ir')
if len(ir_cameras) == 0:
    print("ERROR: IR camera not available. Check CARLA build includes IR sensor.")
```

### No Data on ROS Topics

1. Verify ROS2 bridge is running
2. Check topic names match your vehicle ID
3. Ensure `ENABLE_ROS2=ON` was set during build

```bash
# Check if ROS2 bridge is publishing
ros2 node list | grep carla

# Verify topic exists
ros2 topic list | grep ir_camera
```

### Performance Issues

- Reduce resolution: Use 80x60 or lower
- Increase sensor_tick: Use 0.2 (5 Hz) or higher
- Limit number of active IR cameras
- Check GPU utilization with `nvidia-smi`

## References

- [FLIR Lepton 3.5 Datasheet](https://www.flir.com/products/lepton/)
- [CARLA Sensors Documentation](https://carla.readthedocs.io/en/latest/ref_sensors/)
- [ROS2 sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)
- [ROS2 sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)

## Support

For issues or questions:
- CARLA IR Camera: Open an issue on this repository
- CARLA Core: [CARLA GitHub Issues](https://github.com/carla-simulator/carla/issues)
- ROS2 Integration: [CARLA ROS Bridge](https://github.com/carla-simulator/ros-bridge)
