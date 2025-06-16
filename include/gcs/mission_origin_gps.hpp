#pragma once
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace gcs {

using namespace std::chrono_literals;
class MissionOriginGPS : public rclcpp::Node {
 public:
  explicit MissionOriginGPS(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("mission_origin_gps", options) {
    // Declare and get parameters
    this->declare_parameter<double>("mission_origin_lon", 0.0);
    this->get_parameter("mission_origin_lon", lon_origin);

    this->declare_parameter<double>("mission_origin_lat", 0.0);
    this->get_parameter("mission_origin_lat", lat_origin);

    this->declare_parameter<double>("heading", 2.4);
    this->get_parameter("heading", heading);

    // Set up QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                           qos_profile);

    // Create publishers
    publisher_gps_ = this->create_publisher<geometry_msgs::msg::Point>(
        "mission_origin_gps", qos);

    // Pre-create the GPS message since data is static
    gps_message_.x = lon_origin;
    gps_message_.y = lat_origin;
    gps_message_.z = heading;

    // Create a timer to publish messages periodically (reduced frequency for
    // static data)
    timer_ = this->create_wall_timer(2000ms, [this]() -> void {
      // Publish static GPS data (every 5 seconds is sufficient)
      publisher_gps_->publish(gps_message_);
    });
  }

 private:
  // Member variables
  double lon_origin;
  double lat_origin;
  double heading;  // Fixed type inconsistency - should be double like others

  // Pre-created message to avoid repeated allocation
  geometry_msgs::msg::Point gps_message_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_gps_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace gcs
