#include "gcs/status_pac.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StatusPAC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
