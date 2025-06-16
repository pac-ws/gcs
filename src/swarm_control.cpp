#include "gcs/swarm_control.hpp"

SwarmControl::SwarmControl() : Node("swarm_control"), qos_(1) {

    // QoS
    RCLCPP_DEBUG(this->get_logger(), "Creating Node");
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    qos_ = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, 10),
    qos_profile);

    timer_ = this->create_wall_timer(
    100ms, std::bind(&SwarmControl::Update, this));

    initscr();

    id_col_ = {"ID", COL_SINGLE_WIDTH};
    status_col_ = {"STATUS", COL_SINGLE_WIDTH};
    batt_col_ = {"BATT", COL_SINGLE_WIDTH};
    lat_col_ = {"LAT", COL_DOUBLE_WIDTH};
    lon_col_ = {"LON", COL_DOUBLE_WIDTH};
    sats_col_ = {"SATS", COL_SINGLE_WIDTH};
    //pose_col_ = {"POSE", COL_QUAD_WIDTH};

    columns_ = {id_col_, status_col_, batt_col_, lat_col_, lon_col_, sats_col_}
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
            std::string robot_name = topic_name.substr(1, topic_name.find("/status") - 1); std::string topic = "/" + robot_name + "/status";
            rclcpp::Subscription<async_pac_gnn_interfaces::msg::RobotStatus>::SharedPtr sub 
                = this->create_subscription<async_pac_gnn_interfaces::msg::RobotStatus>( topic, qos_,
                        [this, topic_name](const async_pac_gnn_interfaces::msg::RobotStatus::SharedPtr msg){
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

void SwarmControl::GetRobotStatus() {
    for (auto i = robot_status_.begin(); i != robot_status_.end(); i++) {
        auto px = i->second.local_pos_x;
        auto py = i->second.local_pos_y;
        auto pz = i->second.local_pos_z;
        
        RCLCPP_INFO(this->get_logger(), "local pos: (%f, %f, %f)", px, py, pz);
    }
}

void SwarmControl::StatusTable(TermPoint2D top_left, int row_height, int col_width){

    int height = row_height * robot_status_.size() + 4;
    int width = col_width * 18;

    WINDOW* table = newwin(height, 120, 0, 0);
    box(table, 0, 0);
    //mvwprintw(table, 1, 2, "Ready | ID | Status | Batt | Breach | LAT | LON | SAT | Local X | Local Y | Local Z | Pose X | Pose Y | Pose Z");
    //mvwprintw(table, 2, 2, "------+----+--------+------+--------+-----+-----+-----+---------+---------+---------+--------+--------+-------");

    size_t cnt = 0;
    for (auto i = robot_status_.begin(); i != robot_status_.end(); i++, cnt++) {
        auto ready = i->second.ready;
        auto batt = i->second.batt;
        auto state = i->second.state;
        auto breach = i->second.breach;
        auto gps_lat = i->second.gps_lat;
        auto gps_lon = i->second.gps_lon;
        auto gps_alt = i->second.gps_alt;
        auto gps_sats = i->second.gps_sats;
        auto gps_heading = i->second.gps_heading;
        auto local_pos_x = i->second.local_pos_x;
        auto local_pos_y = i->second.local_pos_y;
        auto local_pos_z = i->second.local_pos_z;
        auto local_pos_heading = i->second.local_pos_heading;
        auto pose_x = i->second.pose_x;
        auto pose_y = i->second.pose_y;
        auto pose_z = i->second.pose_z;
        auto ned_vel_x = i->second.ned_vel_x;
        auto ned_vel_y = i->second.ned_vel_y;
        auto ned_vel_z = i->second.ned_vel_z;

        columns_[0].data.push_back(cnt);
        columns_[1].data.push_back(i->second.state);
        columns_[2].data.push_back(i->second.batt);
        columns_[3].data.push_back(i->second.lat);
        columns_[4].data.push_back(i->second.lon);
        columns_[5].data.push_back(i->second.sats);
    }
    
    int start = 2;
    for (auto i = columns_.begin(); i != columns_.end(); i++) {
        DrawColumn(*i, start);
        start += i->width;
    }
    wrefresh(table);
}

void SwarmControl::DrawColumn(WINDOW table, TableColumn col, int start){
    auto data = col.data;
    mvwprintw(table, 1, start, col.header);
    size_t row = 3;
    for (auto i = data.begin(); i != data.end(); i++, row++) {
        mvwprintw(table, row, start, i);
    }
}

void SwarmControl::Update() {
    GetRobotSubs();
    //GetRobotStatus();

    //clear();
    TermPoint2D table_top_left = {.x = 0, .y = 0};
    StatusTable(table_top_left, 1, 50);
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
