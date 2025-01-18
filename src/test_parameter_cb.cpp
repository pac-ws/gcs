#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;
class TestParameterCB : public rclcpp::Node {
 private:
  bool enable_ = false;
  bool takeoff_ = false;
  bool land_ = false;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::ParameterEventHandler> mission_control_PEH_ptr_;
  rclcpp::ParameterCallbackHandle::SharedPtr handle_;
  rclcpp::SyncParametersClient::SharedPtr sync_parameters_client_;
/* SyncParametersClient (rclcpp::Node::SharedPtr node, const std::string &remote_node_name="", const rmw_qos_profile_t &qos_profile=rmw_qos_profile_parameters) */
 public:
  TestParameterCB() : Node("test_parameter_cb") {

    sync_parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "mission_control");
    while (!sync_parameters_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto result = sync_parameters_client_->get_parameters({"enable"});
    RCLCPP_INFO(this->get_logger(), "result size = %d", result.size());
    for (auto &p : result) {
      if (p.get_name() == "enable") {
        enable_ = p.get_value<bool>();
        RCLCPP_INFO(this->get_logger(), "init enable = %d", enable_);
      }
    }
    timer_ = this->create_wall_timer(1000ms,std::bind(&TestParameterCB::timer_callback, this));

    mission_control_PEH_ptr_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto cb2 = [this](const rclcpp::Parameter& p) {
      enable_ = p.get_value<bool>();
      RCLCPP_INFO(this->get_logger(), "received enable = %d", enable_);
    };
    handle_= mission_control_PEH_ptr_->add_parameter_callback(
        "enable", cb2, "mission_control");
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "cb_enable_ = %d", enable_);
  }

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParameterCB>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
