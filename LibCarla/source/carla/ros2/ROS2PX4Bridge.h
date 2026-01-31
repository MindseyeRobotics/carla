// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <memory>
#include <string>

namespace carla {
  namespace geom {
    struct Vector3D;
    class Transform;
    class GeoLocation;
  }
}

namespace carla {
namespace ros2 {

  struct ROS2PX4BridgeImpl;

  class ROS2PX4Bridge {
    public:
      ROS2PX4Bridge(void* vehicle, const char* ros_name = "", const char* parent = "");
      ~ROS2PX4Bridge();
      ROS2PX4Bridge(const ROS2PX4Bridge&);
      ROS2PX4Bridge& operator=(const ROS2PX4Bridge&);
      ROS2PX4Bridge(ROS2PX4Bridge&&);
      ROS2PX4Bridge& operator=(ROS2PX4Bridge&&);

      bool Init();
      void Destroy();
      
      // Publishers - Send vehicle state to PX4
      void PublishOdometry(
          const carla::geom::Transform& transform,
          const carla::geom::Vector3D& velocity,
          const carla::geom::Vector3D& angular_velocity);
      
      void PublishIMU(
          const carla::geom::Vector3D& accelerometer,
          const carla::geom::Vector3D& gyroscope,
          float compass);
      
      void PublishGPS(const carla::geom::GeoLocation& location);
      
      // Subscribers - Receive control from PX4
      bool HasNewActuatorControls();
      void GetActuatorControls(float& throttle, float& roll, float& pitch, float& yaw);
      
      void* GetVehicle();
      bool IsAlive();

      void SetFrame(uint64_t frame);
      void SetTimestamp(double timestamp);

    private:
      void CoordinateTransform(
          const carla::geom::Vector3D& in,
          float& x, float& y, float& z);
      
      void QuaternionFromYaw(float yaw, float& w, float& x, float& y, float& z);

    private:
      std::shared_ptr<ROS2PX4BridgeImpl> _impl;
      std::string _name;
      std::string _parent;
      uint64_t _frame;
      int32_t _seconds;
      uint32_t _nanoseconds;
  };
}
}
