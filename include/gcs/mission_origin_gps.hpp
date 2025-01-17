#pragma once
#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

class MissionOriginGPS : public rclcpp::Node {
public:
  MissionOriginGPS() : Node("mission_origin_gps") {
    // Declare and get parameters
    this->declare_parameter<double>("mission_origin_lon", 0.0);
    this->get_parameter("mission_origin_lon", lon_origin);

    this->declare_parameter<double>("mission_origin_lat", 0.0);
    this->get_parameter("mission_origin_lat", lat_origin);

    this->declare_parameter<double>("heading", 2.4);
    this->get_parameter("heading", heading);

    // Set up QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Create publishers
    publisher_gps_ = this->create_publisher<geometry_msgs::msg::Point>(
        "mission_origin_gps", qos);

    // Create a timer to publish messages periodically
    timer_ = this->create_wall_timer(
        500ms, [this]() -> void {
          // Publish GPS data
          auto message_gps = geometry_msgs::msg::Point();
          message_gps.x = lon_origin;
          message_gps.y = lat_origin;
          message_gps.z = heading;
          publisher_gps_->publish(message_gps);
        });
  }

private:
  // Member variables
  double lon_origin;
  double lat_origin;
  float heading;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_gps_;
  rclcpp::TimerBase::SharedPtr timer_;
};
