// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "ROS2PX4Bridge.h"

#include "carla/geom/Vector3D.h"
#include "carla/geom/Transform.h"
#include "carla/geom/GeoLocation.h"

#include "carla/ros2/types/Odometry.h"
#include "carla/ros2/types/OdometryPubSubTypes.h"
#include "carla/ros2/types/Imu.h"
#include "carla/ros2/types/ImuPubSubTypes.h"
#include "carla/ros2/types/NavSatFix.h"
#include "carla/ros2/types/NavSatFixPubSubTypes.h"
#include "carla/ros2/types/PX4ActuatorControls.h"
#include "carla/ros2/types/PX4ActuatorControlsPubSubTypes.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/qos/QosPolicies.h>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>

#include <iostream>
#include <cmath>

namespace carla {
namespace ros2 {

  namespace efd = eprosima::fastdds::dds;
  using erc = eprosima::fastrtps::types::ReturnCode_t;

  // Conversion constants
  constexpr float DEG_TO_RAD = M_PI / 180.0f;
  constexpr float HALF = 0.5f;

  struct ROS2PX4BridgeImpl {
    efd::DomainParticipant* _participant { nullptr };
    
    // Publishers
    efd::Publisher* _publisher { nullptr };
    efd::Topic* _odometry_topic { nullptr };
    efd::DataWriter* _odometry_writer { nullptr };
    efd::TypeSupport _odometry_type { new nav_msgs::msg::OdometryPubSubType() };
    nav_msgs::msg::Odometry _odometry_msg {};
    
    efd::Topic* _imu_topic { nullptr };
    efd::DataWriter* _imu_writer { nullptr };
    efd::TypeSupport _imu_type { new sensor_msgs::msg::ImuPubSubType() };
    sensor_msgs::msg::Imu _imu_msg {};
    
    efd::Topic* _gps_topic { nullptr };
    efd::DataWriter* _gps_writer { nullptr };
    efd::TypeSupport _gps_type { new sensor_msgs::msg::NavSatFixPubSubType() };
    sensor_msgs::msg::NavSatFix _gps_msg {};
    
    // Subscribers
    efd::Subscriber* _subscriber { nullptr };
    efd::Topic* _actuator_topic { nullptr };
    efd::DataReader* _actuator_reader { nullptr };
    efd::TypeSupport _actuator_type { new px4_msgs::msg::PX4ActuatorControlsPubSubType() };
    px4_msgs::msg::PX4ActuatorControls _actuator_msg {};
    
    bool _new_actuator_message { false };
    bool _alive { true };
    void* _vehicle { nullptr };
  };

  bool ROS2PX4Bridge::Init() {
    if (_impl->_odometry_type == nullptr || _impl->_imu_type == nullptr || 
        _impl->_gps_type == nullptr || _impl->_actuator_type == nullptr) {
        std::cerr << "Invalid TypeSupport" << std::endl;
        return false;
    }

    // Create participant
    efd::DomainParticipantQos pqos = efd::PARTICIPANT_QOS_DEFAULT;
    pqos.name(_name + "_px4_bridge");
    auto factory = efd::DomainParticipantFactory::get_instance();
    _impl->_participant = factory->create_participant(0, pqos);
    if (_impl->_participant == nullptr) {
        std::cerr << "Failed to create DomainParticipant" << std::endl;
        return false;
    }

    // Register types
    _impl->_odometry_type.register_type(_impl->_participant);
    _impl->_imu_type.register_type(_impl->_participant);
    _impl->_gps_type.register_type(_impl->_participant);
    _impl->_actuator_type.register_type(_impl->_participant);

    // Create publisher
    efd::PublisherQos pubqos = efd::PUBLISHER_QOS_DEFAULT;
    _impl->_publisher = _impl->_participant->create_publisher(pubqos, nullptr);
    if (_impl->_publisher == nullptr) {
      std::cerr << "Failed to create Publisher" << std::endl;
      return false;
    }

    // Create subscriber
    efd::SubscriberQos subqos = efd::SUBSCRIBER_QOS_DEFAULT;
    _impl->_subscriber = _impl->_participant->create_subscriber(subqos, nullptr);
    if (_impl->_subscriber == nullptr) {
      std::cerr << "Failed to create Subscriber" << std::endl;
      return false;
    }

    // Create topics and writers/readers
    efd::TopicQos tqos = efd::TOPIC_QOS_DEFAULT;
    
    // Odometry topic - publish to PX4
    std::string odom_topic = "/fmu/in/vehicle_odometry";
    _impl->_odometry_topic = _impl->_participant->create_topic(odom_topic, _impl->_odometry_type->getName(), tqos);
    if (_impl->_odometry_topic == nullptr) {
        std::cerr << "Failed to create Odometry Topic" << std::endl;
        return false;
    }

    efd::DataWriterQos wqos = efd::DATAWRITER_QOS_DEFAULT;
    _impl->_odometry_writer = _impl->_publisher->create_datawriter(_impl->_odometry_topic, wqos);
    if (_impl->_odometry_writer == nullptr) {
        std::cerr << "Failed to create Odometry DataWriter" << std::endl;
        return false;
    }

    // IMU topic - publish to PX4
    std::string imu_topic = "/fmu/in/vehicle_imu";
    _impl->_imu_topic = _impl->_participant->create_topic(imu_topic, _impl->_imu_type->getName(), tqos);
    if (_impl->_imu_topic == nullptr) {
        std::cerr << "Failed to create IMU Topic" << std::endl;
        return false;
    }

    _impl->_imu_writer = _impl->_publisher->create_datawriter(_impl->_imu_topic, wqos);
    if (_impl->_imu_writer == nullptr) {
        std::cerr << "Failed to create IMU DataWriter" << std::endl;
        return false;
    }

    // GPS topic - publish to PX4
    std::string gps_topic = "/fmu/in/vehicle_gps_position";
    _impl->_gps_topic = _impl->_participant->create_topic(gps_topic, _impl->_gps_type->getName(), tqos);
    if (_impl->_gps_topic == nullptr) {
        std::cerr << "Failed to create GPS Topic" << std::endl;
        return false;
    }

    _impl->_gps_writer = _impl->_publisher->create_datawriter(_impl->_gps_topic, wqos);
    if (_impl->_gps_writer == nullptr) {
        std::cerr << "Failed to create GPS DataWriter" << std::endl;
        return false;
    }

    // Actuator controls topic - subscribe from PX4
    std::string actuator_topic = "/fmu/out/actuator_controls_0";
    _impl->_actuator_topic = _impl->_participant->create_topic(actuator_topic, _impl->_actuator_type->getName(), tqos);
    if (_impl->_actuator_topic == nullptr) {
        std::cerr << "Failed to create Actuator Topic" << std::endl;
        return false;
    }

    efd::DataReaderQos rqos = efd::DATAREADER_QOS_DEFAULT;
    _impl->_actuator_reader = _impl->_subscriber->create_datareader(_impl->_actuator_topic, rqos, nullptr);
    if (_impl->_actuator_reader == nullptr) {
        std::cerr << "Failed to create Actuator DataReader" << std::endl;
        return false;
    }

    return true;
  }

  void ROS2PX4Bridge::PublishOdometry(
      const carla::geom::Transform& transform,
      const carla::geom::Vector3D& velocity,
      const carla::geom::Vector3D& angular_velocity) {
    
    // Set header
    _impl->_odometry_msg.header().stamp().sec(_seconds);
    _impl->_odometry_msg.header().stamp().nanosec(_nanoseconds);
    _impl->_odometry_msg.header().frame_id("map");
    _impl->_odometry_msg.child_frame_id("base_link");

    // Convert CARLA coordinates (UE4: X forward, Y right, Z up) to ROS (X forward, Y left, Z up)
    auto location = transform.location;
    _impl->_odometry_msg.pose().pose().position().x(location.x);
    _impl->_odometry_msg.pose().pose().position().y(-location.y);
    _impl->_odometry_msg.pose().pose().position().z(location.z);

    // Convert rotation to quaternion
    auto rotation = transform.rotation;
    float yaw = rotation.yaw * DEG_TO_RAD;
    float pitch = rotation.pitch * DEG_TO_RAD;
    float roll = rotation.roll * DEG_TO_RAD;

    // Convert to quaternion (ZYX intrinsic rotations)
    float cy = std::cos(yaw * HALF);
    float sy = std::sin(yaw * HALF);
    float cp = std::cos(pitch * HALF);
    float sp = std::sin(pitch * HALF);
    float cr = std::cos(roll * HALF);
    float sr = std::sin(roll * HALF);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    _impl->_odometry_msg.pose().pose().orientation().w(qw);
    _impl->_odometry_msg.pose().pose().orientation().x(qx);
    _impl->_odometry_msg.pose().pose().orientation().y(-qy);
    _impl->_odometry_msg.pose().pose().orientation().z(-qz);

    // Velocity in body frame
    _impl->_odometry_msg.twist().twist().linear().x(velocity.x);
    _impl->_odometry_msg.twist().twist().linear().y(-velocity.y);
    _impl->_odometry_msg.twist().twist().linear().z(velocity.z);

    // Angular velocity
    _impl->_odometry_msg.twist().twist().angular().x(angular_velocity.x);
    _impl->_odometry_msg.twist().twist().angular().y(-angular_velocity.y);
    _impl->_odometry_msg.twist().twist().angular().z(angular_velocity.z);

    _impl->_odometry_writer->write(&_impl->_odometry_msg);
  }

  void ROS2PX4Bridge::PublishIMU(
      const carla::geom::Vector3D& accelerometer,
      const carla::geom::Vector3D& gyroscope,
      float compass) {
    
    // Set header
    _impl->_imu_msg.header().stamp().sec(_seconds);
    _impl->_imu_msg.header().stamp().nanosec(_nanoseconds);
    _impl->_imu_msg.header().frame_id("imu_link");

    // Convert orientation from compass
    float yaw = compass * DEG_TO_RAD;
    float qw, qx, qy, qz;
    QuaternionFromYaw(yaw, qw, qx, qy, qz);
    
    _impl->_imu_msg.orientation().w(qw);
    _impl->_imu_msg.orientation().x(qx);
    _impl->_imu_msg.orientation().y(qy);
    _impl->_imu_msg.orientation().z(qz);

    // Angular velocity (convert to ROS coordinates)
    _impl->_imu_msg.angular_velocity().x(gyroscope.x);
    _impl->_imu_msg.angular_velocity().y(-gyroscope.y);
    _impl->_imu_msg.angular_velocity().z(gyroscope.z);

    // Linear acceleration (convert to ROS coordinates)
    _impl->_imu_msg.linear_acceleration().x(accelerometer.x);
    _impl->_imu_msg.linear_acceleration().y(-accelerometer.y);
    _impl->_imu_msg.linear_acceleration().z(accelerometer.z);

    _impl->_imu_writer->write(&_impl->_imu_msg);
  }

  void ROS2PX4Bridge::PublishGPS(const carla::geom::GeoLocation& location) {
    // Set header
    _impl->_gps_msg.header().stamp().sec(_seconds);
    _impl->_gps_msg.header().stamp().nanosec(_nanoseconds);
    _impl->_gps_msg.header().frame_id("gps");

    _impl->_gps_msg.latitude(location.latitude);
    _impl->_gps_msg.longitude(location.longitude);
    _impl->_gps_msg.altitude(location.altitude);
    
    _impl->_gps_msg.status().status(0); // STATUS_FIX
    _impl->_gps_msg.status().service(1); // SERVICE_GPS

    _impl->_gps_writer->write(&_impl->_gps_msg);
  }

  bool ROS2PX4Bridge::HasNewActuatorControls() {
    if (!_impl->_actuator_reader) {
        return false;
    }

    efd::SampleInfo info;
    eprosima::fastrtps::types::ReturnCode_t rcode = 
        _impl->_actuator_reader->take_next_sample(&_impl->_actuator_msg, &info);
    
    if (rcode == erc::ReturnCodeValue::RETCODE_OK) {
        _impl->_new_actuator_message = true;
        return true;
    }
    
    return false;
  }

  void ROS2PX4Bridge::GetActuatorControls(float& throttle, float& roll, float& pitch, float& yaw) {
    if (_impl->_new_actuator_message) {
        // PX4 actuator controls mapping:
        // control[0] = roll
        // control[1] = pitch
        // control[2] = yaw
        // control[3] = throttle
        const auto& controls = _impl->_actuator_msg.control();
        roll = controls[0];
        pitch = controls[1];
        yaw = controls[2];
        throttle = controls[3];
        
        _impl->_new_actuator_message = false;
    } else {
        throttle = 0.0f;
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
  }

  void ROS2PX4Bridge::Destroy() {
    _impl->_alive = false;
  }

  void* ROS2PX4Bridge::GetVehicle() {
    return _impl->_vehicle;
  }

  bool ROS2PX4Bridge::IsAlive() {
    return _impl->_alive;
  }

  void ROS2PX4Bridge::SetFrame(uint64_t frame) {
    _frame = frame;
  }

  void ROS2PX4Bridge::SetTimestamp(double timestamp) {
    // Note: NANOS_PER_SECOND constant also defined in ROS2.cpp for consistency
    constexpr double NANOS_PER_SECOND = 1e9;
    _seconds = static_cast<int32_t>(timestamp);
    _nanoseconds = static_cast<uint32_t>((timestamp - _seconds) * NANOS_PER_SECOND);
  }

  void ROS2PX4Bridge::CoordinateTransform(
      const carla::geom::Vector3D& in,
      float& x, float& y, float& z) {
    // CARLA uses UE4 coordinates (X forward, Y right, Z up)
    // ROS uses (X forward, Y left, Z up)
    x = in.x;
    y = -in.y;
    z = in.z;
  }

  void ROS2PX4Bridge::QuaternionFromYaw(float yaw, float& w, float& x, float& y, float& z) {
    // Simplified yaw-only quaternion for compass-based orientation (IMU)
    // For full 6DOF orientation with roll/pitch/yaw, see PublishOdometry()
    float half_yaw = yaw * HALF;
    w = std::cos(half_yaw);
    x = 0.0f;
    y = 0.0f;
    z = std::sin(half_yaw);
  }

  ROS2PX4Bridge::ROS2PX4Bridge(void* vehicle, const char* ros_name, const char* parent) :
      _impl(std::make_shared<ROS2PX4BridgeImpl>()),
      _frame(0),
      _seconds(0),
      _nanoseconds(0) {
    _impl->_vehicle = vehicle;
    _name = ros_name;
    _parent = parent;
  }

  ROS2PX4Bridge::~ROS2PX4Bridge() {
      if (!_impl)
          return;

      if (_impl->_odometry_writer)
          _impl->_publisher->delete_datawriter(_impl->_odometry_writer);

      if (_impl->_imu_writer)
          _impl->_publisher->delete_datawriter(_impl->_imu_writer);

      if (_impl->_gps_writer)
          _impl->_publisher->delete_datawriter(_impl->_gps_writer);

      if (_impl->_actuator_reader)
          _impl->_subscriber->delete_datareader(_impl->_actuator_reader);

      if (_impl->_publisher)
          _impl->_participant->delete_publisher(_impl->_publisher);

      if (_impl->_subscriber)
          _impl->_participant->delete_subscriber(_impl->_subscriber);

      if (_impl->_odometry_topic)
          _impl->_participant->delete_topic(_impl->_odometry_topic);

      if (_impl->_imu_topic)
          _impl->_participant->delete_topic(_impl->_imu_topic);

      if (_impl->_gps_topic)
          _impl->_participant->delete_topic(_impl->_gps_topic);

      if (_impl->_actuator_topic)
          _impl->_participant->delete_topic(_impl->_actuator_topic);

      if (_impl->_participant)
          efd::DomainParticipantFactory::get_instance()->delete_participant(_impl->_participant);
  }

  ROS2PX4Bridge::ROS2PX4Bridge(const ROS2PX4Bridge& other) {
    _name = other._name;
    _parent = other._parent;
    _frame = other._frame;
    _seconds = other._seconds;
    _nanoseconds = other._nanoseconds;
    _impl = other._impl;
  }

  ROS2PX4Bridge& ROS2PX4Bridge::operator=(const ROS2PX4Bridge& other) {
    _name = other._name;
    _parent = other._parent;
    _frame = other._frame;
    _seconds = other._seconds;
    _nanoseconds = other._nanoseconds;
    _impl = other._impl;
    return *this;
  }

  ROS2PX4Bridge::ROS2PX4Bridge(ROS2PX4Bridge&& other) {
    _name = std::move(other._name);
    _parent = std::move(other._parent);
    _frame = other._frame;
    _seconds = other._seconds;
    _nanoseconds = other._nanoseconds;
    _impl = std::move(other._impl);
  }

  ROS2PX4Bridge& ROS2PX4Bridge::operator=(ROS2PX4Bridge&& other) {
    _name = std::move(other._name);
    _parent = std::move(other._parent);
    _frame = other._frame;
    _seconds = other._seconds;
    _nanoseconds = other._nanoseconds;
    _impl = std::move(other._impl);
    return *this;
  }

}
}
