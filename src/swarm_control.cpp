#include "gcs/swarm_control.hpp"

SwarmControl::SwarmControl() : Node("swarm_control"), qos_(1) {
    // QoS
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_ = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, 10),
    qos_profile);
    
    // Curses
    initscr();
    InitializeUI();

    // Main Loop Timer
    timer_ = this->create_wall_timer(
    100ms, std::bind(&SwarmControl::Update, this));

    TableColumn<std::string> test_col(status_table_, "Test", std::vector<std::string>(), 10, 0);
}

void SwarmControl::InitializeUI() {
    status_table_ = newwin(STATUS_TABLE_HEIGHT, STATUS_TABLE_WIDTH, STATUS_TABLE_X, STATUS_TABLE_Y);
    box(status_table_, 0, 0);

    id_col_     = std::make_shared<TableColumn<std::string>>(status_table_, "ID", std::vector<std::string>(), COL_SINGLE_WIDTH, DATA_ROW_START);
    state_col_ = std::make_shared<TableColumn<std::string>>(status_table_, "STATUS",std::vector<std::string>(), COL_SINGLE_WIDTH, DATA_ROW_START);
    batt_col_   = std::make_shared<TableColumn<int>>(status_table_, "BATT", std::vector<int>(), COL_SINGLE_WIDTH, DATA_ROW_START);
    lat_col_    = std::make_shared<TableColumn<float>>(status_table_, "LAT", std::vector<float>(), COL_DOUBLE_WIDTH, DATA_ROW_START);
    lon_col_    = std::make_shared<TableColumn<float>>(status_table_, "LON", std::vector<float>(), COL_DOUBLE_WIDTH, DATA_ROW_START);
    sats_col_   = std::make_shared<TableColumn<int>>(status_table_, "SATS", std::vector<int>(), COL_SINGLE_WIDTH, DATA_ROW_START);

    columns_ = {id_col_, state_col_, batt_col_, lat_col_, lon_col_, sats_col_};
}

void SwarmControl::GetRobotSubs() {
    auto topic_map = this->get_topic_names_and_types();
    RCLCPP_DEBUG(this->get_logger(), "Getting Robot Topics");
    for (const auto &pair : topic_map) {
        const std::string &topic_name = pair.first;
        const auto &types = pair.second;

        RCLCPP_DEBUG(this->get_logger(), "topic_name: %s", topic_name.c_str());
        
        if (robot_status_subs_.count(topic_name) > 0) {
            continue;
        }

        if (topic_name.find("/status") != std::string::npos &&
                std::find(types.begin(), types.end(), "async_pac_gnn_interfaces/msg/RobotStatus") != types.end()) {
            std::string robot_name = topic_name.substr(1, topic_name.find("/status") - 1); 
            std::string topic = "/" + robot_name + "/status";
            rclcpp::Subscription<async_pac_gnn_interfaces::msg::RobotStatus>::SharedPtr sub 
                = this->create_subscription<async_pac_gnn_interfaces::msg::RobotStatus>( topic, qos_,
                        [this, topic_name, robot_name](const async_pac_gnn_interfaces::msg::RobotStatus::SharedPtr msg){
                            robot_status_[topic_name].id = robot_name;
                            robot_status_[topic_name].ready = msg->ready;
                            robot_status_[topic_name].batt = msg->batt;
                            robot_status_[topic_name].state = msg->state;
                            robot_status_[topic_name].breach = msg->breach;
                            robot_status_[topic_name].gps_lat = msg->gps_lat;
                            robot_status_[topic_name].gps_lon = msg->gps_lon;
                            robot_status_[topic_name].gps_alt = msg->gps_alt;
                            robot_status_[topic_name].gps_sats = msg->gps_sats;
                            robot_status_[topic_name].gps_heading = msg->gps_heading;
                            robot_status_[topic_name].local_pos_x = msg->local_pos_x;
                            robot_status_[topic_name].local_pos_y = msg->local_pos_y;
                            robot_status_[topic_name].local_pos_z = msg->local_pos_z;
                            robot_status_[topic_name].local_pos_heading = msg->local_pos_heading;
                            robot_status_[topic_name].pose_x = msg->pose_x;
                            robot_status_[topic_name].pose_y = msg->pose_y;
                            robot_status_[topic_name].pose_z = msg->pose_z;
                            robot_status_[topic_name].ned_vel_x = msg->ned_vel_x;
                            robot_status_[topic_name].ned_vel_y = msg->ned_vel_y;
                            robot_status_[topic_name].ned_vel_z = msg->ned_vel_z;
                        });
            robot_status_subs_[topic_name] = sub;
        }
    }
}

void SwarmControl::StatusTable() {
    // Clear out old data
    for (auto i = columns_.begin(); i != columns_.end(); i++) {
        (*i)->ClearData();
    }
        
    // Load latest data
    for (auto i = robot_status_.begin(); i != robot_status_.end(); i++) {
        id_col_->AddRow(i->second.id);
        state_col_->AddRow(i->second.state);
        batt_col_->AddRow(i->second.batt);
        lat_col_->AddRow(i->second.gps_lat);
        lon_col_->AddRow(i->second.gps_lon);
        sats_col_->AddRow(i->second.gps_sats);
    }

    int col_start = 2;
    for (auto i = columns_.begin(); i != columns_.end(); i++) {
        (*i)->DrawColumn(col_start);
        col_start += (*i)->GetWidth();
    }
    wrefresh(status_table_);
}

void SwarmControl::Update() {
    GetRobotSubs();
    StatusTable();
    refresh();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwarmControl>();
    rclcpp::spin(node);
    endwin();			
    rclcpp::shutdown();
    return 0;
}
