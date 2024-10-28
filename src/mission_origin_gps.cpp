#include "gcs/mission_origin_gps.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionOriginGPS>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
