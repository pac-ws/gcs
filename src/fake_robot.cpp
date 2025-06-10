#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class FakeRobot : public rclcpp::Node {
 public:
  FakeRobot() : Node("fake_robot") {
    // Declare parameters
    this->declare_parameter("buffer_size", 10);
    this->declare_parameter("pos_x", 0.0);
    this->declare_parameter("pos_y", 0.0);
    this->declare_parameter("speed_limit", 1.0);

    // Get parameter values
    init_pos_x_ = this->get_parameter("pos_x").as_double();
    init_pos_y_ = this->get_parameter("pos_y").as_double();
    speed_limit_ = this->get_parameter("speed_limit").as_double();

    // Initialize current position
    current_pos_.pose.position.x = init_pos_x_;
    current_pos_.pose.position.y = init_pos_y_;
    current_pos_.pose.position.z = 0.0;
    current_pos_.pose.orientation.x = 0.0;
    current_pos_.pose.orientation.y = 0.0;
    current_pos_.pose.orientation.z = 0.0;
    current_pos_.pose.orientation.w = 1.0;

    // Initialize current velocity
    current_vel_.twist.linear.x = 0.0;
    current_vel_.twist.linear.y = 0.0;
    current_vel_.twist.linear.z = 0.0;
    current_vel_.twist.angular.x = 0.0;
    current_vel_.twist.angular.y = 0.0;
    current_vel_.twist.angular.z = 0.0;

    // Create subscription
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "cmd_vel", qos_profile_,
        std::bind(&FakeRobot::cmd_vel_callback, this, std::placeholders::_1));

    // Create publisher
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "pose", qos_profile_);

    // Create timer (30ms = 0.030s)
    timer_ = this->create_wall_timer(
        30ms, std::bind(&FakeRobot::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "FakeRobot node initialized");
    RCLCPP_INFO(this->get_logger(), "Initial position: x=%.2f, y=%.2f",
                init_pos_x_, init_pos_y_);
    RCLCPP_INFO(this->get_logger(), "Speed limit: %.2f", speed_limit_);
  }

 private:
  // Parameters
  double init_pos_x_;
  double init_pos_y_;
  double speed_limit_;

  // State variables
  geometry_msgs::msg::PoseStamped current_pos_;
  geometry_msgs::msg::TwistStamped current_vel_;

  // ROS2 components
  rmw_qos_profile_t qos_profile_sensor_data_ = rmw_qos_profile_sensor_data;
  rclcpp::QoS qos_profile_ =
      rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_sensor_data_.history,
                                            qos_profile_sensor_data_.depth),
                    qos_profile_sensor_data_);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
  rclcpp::TimerBase::SharedPtr timer_;

  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    double vel_x = msg->twist.linear.x;
    double vel_y = msg->twist.linear.y;
    double vel = std::sqrt(vel_x * vel_x + vel_y * vel_y);

    // Apply speed limit
    if (vel > speed_limit_) {
      vel_x = speed_limit_ * vel_x / vel;
      vel_y = speed_limit_ * vel_y / vel;
    }

    current_vel_.twist.linear.x = vel_x;
    current_vel_.twist.linear.y = vel_y;
  }

  void timer_callback() {
    // Update position based on velocity (30ms time step)
    current_pos_.pose.position.x += current_vel_.twist.linear.x * 0.03;
    current_pos_.pose.position.y += current_vel_.twist.linear.y * 0.03;

    // Update timestamp
    current_pos_.header.stamp = this->get_clock()->now();

    // Publish pose
    publisher_pose_->publish(current_pos_);
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto fake_robot_node = std::make_shared<FakeRobot>();

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  try {
    executor->add_node(fake_robot_node);
    executor->spin();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(fake_robot_node->get_logger(),
                 "Exception occurred in spinning: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(fake_robot_node->get_logger(),
                 "Unknown exception occurred in spinning");
  }

  executor->remove_node(fake_robot_node);
  rclcpp::shutdown();
  return 0;
}
