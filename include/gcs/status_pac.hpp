#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>

using namespace std::chrono_literals;

class StatusPAC : public rclcpp::Node {
 public:
  StatusPAC()
      : Node("status_pac"),
        status_(2)  // Default status is '2' (stop)
  {
    // Create a publisher on the 'status_pac' topic
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/pac_gcs/status_pac", 10);

    // Start a separate thread for user input
    input_thread_ = std::thread(&StatusPAC::inputLoop, this);

    // Timer to publish status every 500ms
    timer_ = this->create_wall_timer(
        500ms, std::bind(&StatusPAC::publishStatus, this));
  }

  ~StatusPAC() {
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

 private:
  void inputLoop() {
    while (rclcpp::ok()) {
      std::cout << "Enter status (0: ready, 1: pause, 2: stop, 3: takeoff, 4: land): "
                << std::endl;
      std::cout << "0: ready | lpac publishes velocity commands" << std::endl;
      std::cout << "1: pause | lpac publishes zero velocity commands"
                << std::endl;
      std::cout << "2: stop  | lpac stops publishing"
                << std::endl;
      std::cout << "3: takeoff | takeoff robots"
                << std::endl;
      std::cout << "4: land | land robots"
                << std::endl;
      int current_status = 2;
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status = status_;
      }
      if (current_status == 0) {
        std::cout << "\033[32mCurrent status: 0 ready\033[0m" << std::endl;
      } else if (current_status == 1) {
        std::cout << "\033[34mCurrent status: 1 pause\033[0m" << std::endl;
      } else if (current_status == 2) {
        std::cout << "\033[31mCurrent status: 2 stop\033[0m" << std::endl;
      } else if (current_status == 3) {
        std::cout << "\033[31mCurrent status: 3 takeoff\033[0m" << std::endl;
      } else if (current_status == 4) {
        std::cout << "\033[31mCurrent status: 4 land\033[0m" << std::endl;
      }

      int input_status;
      std::cin >> input_status;

      if (input_status == 0 || input_status == 1 || input_status == 2 || input_status == 3 || input_status == 4) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_ = input_status;
      } else {
        std::cout << "Invalid input. Please enter 0, 1, 2, 3, or 4." << std::endl;
      }
    }
  }

  void publishStatus() {
    int current_status;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      current_status = status_;
    }
    auto message = std_msgs::msg::Int32();
    message.data = current_status;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread input_thread_;
  std::mutex status_mutex_;
  int status_;
};
