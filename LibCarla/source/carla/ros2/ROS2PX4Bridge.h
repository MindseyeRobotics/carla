// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/ros2/ROS2.h"
#include "carla/geom/Transform.h"
#include "carla/geom/Vector3D.h"

#include <memory>
#include <string>
#include <unordered_map>

namespace carla {
namespace ros2 {

  class CarlaPX4SensorPublisher;
  class CarlaPX4ActuatorSubscriber;

  /// @brief PX4 Bridge for CARLA - Connects PX4 autopilot to CARLA multirotor
  ///
  /// This class bridges PX4 autopilot (via MAVROS) with CARLA's multirotor simulation.
  /// It publishes sensor data (IMU, GPS, barometer) to ROS2 topics that MAVROS
  /// forwards to PX4, and subscribes to actuator commands from PX4 to control
  /// the CARLA multirotor.
  class ROS2PX4Bridge : public ROS2
  {
  public:
    // Singleton pattern
    ROS2PX4Bridge(const ROS2PX4Bridge&) = delete;
    ROS2PX4Bridge& operator=(const ROS2PX4Bridge&) = delete;

    static std::shared_ptr<ROS2PX4Bridge> GetInstance();

    // ROS2 interface implementation
    virtual void Enable(bool enable) override;
    virtual void Shutdown() override;
    virtual bool IsEnabled() override { return _enabled; }
    virtual void SetFrame(uint64_t frame) override;
    virtual void RegisterActor(FActorDescription& Description, std::string RosName, void* Actor) override;
    virtual void RemoveActor(void* Actor) override;

    /// Publish IMU data from CARLA multirotor to PX4
    void PublishIMU(
        void* actor,
        const carla::geom::Vector3D& accelerometer,
        const carla::geom::Vector3D& gyroscope,
        double timestamp);

    /// Publish GPS data from CARLA multirotor to PX4
    void PublishGPS(
        void* actor,
        double latitude,
        double longitude,
        double altitude,
        double timestamp);

    /// Publish barometer data from CARLA multirotor to PX4
    void PublishBarometer(
        void* actor,
        double pressure,
        double temperature,
        double timestamp);

    /// Publish odometry (ground truth) for debugging
    void PublishOdometry(
        void* actor,
        const carla::geom::Transform& transform,
        const carla::geom::Vector3D& velocity,
        double timestamp);

    /// Register callback for receiving actuator commands from PX4
    using ActuatorCallback = std::function<void(const std::vector<float>&)>;
    void RegisterActuatorCallback(void* actor, ActuatorCallback callback);

  private:
    ROS2PX4Bridge();
    ~ROS2PX4Bridge();

    static std::shared_ptr<ROS2PX4Bridge> _instance;

    bool _enabled = false;
    uint64_t _frame = 0;
    double _timestamp = 0.0;

    // Publishers for each actor
    struct ActorPublishers {
      std::shared_ptr<CarlaPX4SensorPublisher> imu_publisher;
      std::shared_ptr<CarlaPX4SensorPublisher> gps_publisher;
      std::shared_ptr<CarlaPX4SensorPublisher> baro_publisher;
      std::shared_ptr<CarlaPX4SensorPublisher> odom_publisher;
      std::shared_ptr<CarlaPX4ActuatorSubscriber> actuator_subscriber;
    };

    std::unordered_map<void*, ActorPublishers> _actor_publishers;
    std::unordered_map<void*, ActuatorCallback> _actuator_callbacks;
  };

} // namespace ros2
} // namespace carla
