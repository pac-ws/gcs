#include <rclcpp/rclcpp.hpp>
#include <gcs/mission_control.hpp>
#include <gcs/mission_origin_gps.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);          // zero-copy

  auto mission_control = std::make_shared<gcs::MissionControl>(opts);
  auto origin_gps      = std::make_shared<gcs::MissionOriginGPS>(opts);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(mission_control);
  exec.add_node(origin_gps);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
