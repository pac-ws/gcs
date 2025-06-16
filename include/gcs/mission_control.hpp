#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace gcs {

using namespace std::chrono_literals;
class MissionControl : public rclcpp::Node {
 private:
  bool hardware_enable_ = false;
  bool enable_ = false;
  bool takeoff_ = false;
  bool land_ = false;
  bool geofence_ = false;
  rclcpp::SyncParametersClient::SharedPtr sync_parameters_client_;
  std::shared_ptr<rclcpp::ParameterEventHandler> mission_control_PEH_ptr_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_hardware_enable_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_enable_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_takeoff_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_land_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_geofence_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;

 public:
  explicit MissionControl(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("mission_control", options) {

    this->declare_parameter<bool>("hardware_enable", false);
    this->get_parameter("hardware_enable", hardware_enable_);

    this->declare_parameter<bool>("enable", false);
    this->get_parameter("enable", enable_);

    this->declare_parameter<bool>("takeoff", false);
    this->get_parameter("takeoff", takeoff_);

    this->declare_parameter<bool>("land", false);
    this->get_parameter("land", land_);

    this->declare_parameter<bool>("geofence", false);
    this->get_parameter("geofence", geofence_);

    sync_parameters_client_ =
        std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!sync_parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(),
                  "service not available, waiting again...");
    }

    mission_control_PEH_ptr_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    handle_hardware_enable_ = mission_control_PEH_ptr_->add_parameter_callback(
        "hardware_enable", [this](const rclcpp::Parameter &p) {
          hardware_enable_ = p.get_value<bool>();
        });
    handle_enable_ = mission_control_PEH_ptr_->add_parameter_callback(
        "enable",
        [this](const rclcpp::Parameter &p) { enable_ = p.get_value<bool>(); });
    handle_takeoff_ = mission_control_PEH_ptr_->add_parameter_callback(
        "takeoff",
        [this](const rclcpp::Parameter &p) { takeoff_ = p.get_value<bool>(); });
    handle_land_ = mission_control_PEH_ptr_->add_parameter_callback(
        "land",
        [this](const rclcpp::Parameter &p) { land_ = p.get_value<bool>(); });
    handle_geofence_ = mission_control_PEH_ptr_->add_parameter_callback(
        "geofence", [this](const rclcpp::Parameter &p) {
          geofence_ = p.get_value<bool>();
        });
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "mission_control", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      std_msgs::msg::Int32MultiArray msg;
      msg.data = {hardware_enable_, enable_, takeoff_, land_, geofence_};
      publisher_->publish(msg);
    });
  }
};

}  // namespace gcs
