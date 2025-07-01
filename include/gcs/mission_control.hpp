#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <async_pac_gnn_interfaces/msg/mission_control.hpp>

namespace gcs {

using namespace std::chrono_literals;
class MissionControl : public rclcpp::Node {
 private:
  bool hw_enable_ = false;
  bool ob_enable_ = false;
  bool ob_takeoff_ = false;
  bool ob_land_ = false;
  bool geofence_ = false;
  bool pac_offboard_only_ = false;
  bool pac_lpac_l1_ = false;
  bool pac_lpac_l2_ = false;
  rclcpp::SyncParametersClient::SharedPtr sync_parameters_client_;
  std::shared_ptr<rclcpp::ParameterEventHandler> mission_control_PEH_ptr_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_hw_enable_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_ob_enable_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_takeoff_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_land_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_geofence_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_offboard_only_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_lpac_l1_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_lpac_l2_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<async_pac_gnn_interfaces::msg::MissionControl>::SharedPtr publisher_;

 public:
  explicit MissionControl(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("mission_control", options) {

    this->declare_parameter<bool>("hw_enable", false);
    this->get_parameter("hw_enable", hw_enable_);

    this->declare_parameter<bool>("ob_enable", false);
    this->get_parameter("ob_enable", ob_enable_);

    this->declare_parameter<bool>("ob_takeoff", false);
    this->get_parameter("ob_takeoff", ob_takeoff_);

    this->declare_parameter<bool>("ob_land", false);
    this->get_parameter("ob_land", ob_land_);

    this->declare_parameter<bool>("geofence", false);
    this->get_parameter("geofence", geofence_);

    this->declare_parameter<bool>("pac_offboard_only", false);
    this->get_parameter("pac_offboard_only", pac_offboard_only_);

    this->declare_parameter<bool>("pac_lpac_l1", false);
    this->get_parameter("pac_lpac_l1", pac_lpac_l1_);

    this->declare_parameter<bool>("pac_lpac_l2", false);
    this->get_parameter("pac_lpac_l2", pac_lpac_l2_);

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
    handle_hw_enable_ = mission_control_PEH_ptr_->add_parameter_callback(
        "hw_enable", [this](const rclcpp::Parameter &p) {
          hw_enable_ = p.get_value<bool>();
        });
    handle_ob_enable_ = mission_control_PEH_ptr_->add_parameter_callback(
        "ob_enable",
        [this](const rclcpp::Parameter &p) { ob_enable_ = p.get_value<bool>(); });
    handle_takeoff_ = mission_control_PEH_ptr_->add_parameter_callback(
        "ob_takeoff",
        [this](const rclcpp::Parameter &p) { ob_takeoff_ = p.get_value<bool>(); });
    handle_land_ = mission_control_PEH_ptr_->add_parameter_callback(
        "ob_land",
        [this](const rclcpp::Parameter &p) { ob_land_ = p.get_value<bool>(); });
    handle_geofence_ = mission_control_PEH_ptr_->add_parameter_callback(
        "geofence", [this](const rclcpp::Parameter &p) {
          geofence_ = p.get_value<bool>();
        });
    handle_offboard_only_ = mission_control_PEH_ptr_->add_parameter_callback(
        "pac_offboard_only",
        [this](const rclcpp::Parameter &p) { pac_offboard_only_ = p.get_value<bool>(); });
    handle_lpac_l1_ = mission_control_PEH_ptr_->add_parameter_callback(
        "pac_lpac_l1",
        [this](const rclcpp::Parameter &p) { pac_lpac_l1_ = p.get_value<bool>(); });
    handle_lpac_l2_ = mission_control_PEH_ptr_->add_parameter_callback(
        "pac_lpac_l2",
        [this](const rclcpp::Parameter &p) { pac_lpac_l2_ = p.get_value<bool>(); });
    publisher_ = this->create_publisher<async_pac_gnn_interfaces::msg::MissionControl>(
        "mission_control", 10);
    timer_ = this->create_wall_timer(500ms, [this]() {
      async_pac_gnn_interfaces::msg::MissionControl msg;
      msg.hw_enable = hw_enable_;
      msg.ob_enable = ob_enable_;
      msg.ob_takeoff = ob_takeoff_;
      msg.ob_land = ob_land_;
      msg.geofence = geofence_;
      msg.pac_offboard_only = pac_offboard_only_;
      msg.pac_lpac_l1 = pac_lpac_l1_;
      msg.pac_lpac_l2 = pac_lpac_l2_;
      publisher_->publish(msg);
    });
  }
};

}  // namespace gcs
