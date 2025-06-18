#include <cstdint>
#include <ncurses.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "rcutils/logging.h"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <async_pac_gnn_interfaces/msg/robot_status.hpp>
#include <gcs/table_column.hpp>

using namespace std::chrono_literals;

class SwarmControl : public rclcpp::Node{
    public:
        SwarmControl();
    private:
        struct TermPoint2D {
            int x;
            int y;
        };
        struct Status {
            std::string id;
            uint8_t ready;
            uint8_t batt;
            std::string state;
            bool breach;
            float gps_lat;
            float gps_lon;
            float gps_alt;
            uint8_t gps_sats;
            float gps_heading;
            float local_pos_x;
            float local_pos_y;
            float local_pos_z;
            float local_pos_heading;
            float pose_x;
            float pose_y;
            float pose_z;
            float ned_vel_x;
            float ned_vel_y;
            float ned_vel_z;
        };

        // ROS
        std::unordered_map<std::string, Status> robot_status_;
        std::unordered_map<std::string, rclcpp::Subscription<async_pac_gnn_interfaces::msg::RobotStatus>::SharedPtr> robot_status_subs_;
        rclcpp::QoS qos_;
        rclcpp::TimerBase::SharedPtr timer_;

        // UI Elements
        WINDOW* status_table_;
        const int STATUS_TABLE_HEIGHT = 30;
        const int STATUS_TABLE_WIDTH = 120;
        const int STATUS_TABLE_X = 0;
        const int STATUS_TABLE_Y = 0;

        const int DATA_ROW_START = 3; // Start of table content. Header is at row 1
        const int COL_SINGLE_WIDTH = 10;
        const int COL_DOUBLE_WIDTH = COL_SINGLE_WIDTH * 2;
        const int COL_QUAD_WIDTH = COL_SINGLE_WIDTH * 3;

        std::shared_ptr<TableColumn<std::string>> id_col_;
        std::shared_ptr<TableColumn<std::string>> state_col_;
        std::shared_ptr<TableColumn<int>> batt_col_;
        std::shared_ptr<TableColumn<float>> lat_col_;
        std::shared_ptr<TableColumn<float>> lon_col_;
        std::shared_ptr<TableColumn<int>> sats_col_;
        std::vector<std::shared_ptr<TableColumnBase>> columns_;

        // Methods
        void InitializeUI();
        void GetRobotSubs();
        void StatusTable();
        void Update();
};
