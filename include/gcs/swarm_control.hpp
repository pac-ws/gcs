#include <cstdint>
#include <ncurses.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "rcutils/logging.h"
#include <std_msgs/msg/int32_multi_array.hpp>
#include <async_pac_gnn_interfaces/msg/robot_status.hpp>

using namespace std::chrono_literals;
template<typename T>

class SwarmControl : public rclcpp::Node{
    public:
        SwarmControl();
    private:
        const int COL_SINGLE_WIDTH = 10;
        const int COL_DOUBLE_WIDTH = COL_SINGLE_WIDTH * 2;
        const int COL_QUAD_WIDTH = COL_SINGLE_WIDTH * 3;

        rclcpp::QoS qos_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::unordered_map<std::string, rclcpp::Subscription<async_pac_gnn_interfaces::msg::RobotStatus>::SharedPtr> robot_status_subs_;

        struct TermPoint2D {
            int x;
            int y;
        };
        
        struct TableColumn {
            std::string header;
            int width;
            std::vector<T> data = {};
        };

        TableColumn id_col_;
        TableColumn status_col_;
        TableColumn batt_col_;
        TableColumn lat_col_;
        TableColumn lon_col_;
        TableColumn sats_col_;
        TableColumn pose_col_;

        std::vector<TableColumn> columns_;

        struct Status {
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
        std::unordered_map<std::string, Status> robot_status_;

        void GetRobotSubs();
        void GetRobotStatus();
        void StatusTable(TermPoint2D, int, int);
        void DrawColumn(WINDOW, TableColumn, int);
        void Update();
};
