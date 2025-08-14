#include <visualize_global_path.hpp>

VisualizeGlobal::VisualizeGlobal() : Node("visualize_global")
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_waypoints_marker", 1);

    // raceline.csv 경로로 변경
    std::string raceline_csv_path = ini_load_string("paths", "raceline_csv", "", "/home/jys/ROS2/f1thebeast_ws/src/control_ws/control_real/config/param.ini");
    std::cout << raceline_csv_path << std::endl;
    load_raceline_waypoints(raceline_csv_path);

    // 타이머로 주기적으로 marker publish (예: 0.5초마다)
    marker_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&VisualizeGlobal::publish_global_waypoints_marker, this)
    );

    RCLCPP_INFO(this->get_logger(), "VisualizeGlobal node initialized with raceline data");
}

VisualizeGlobal::~VisualizeGlobal() {
    RCLCPP_INFO(this->get_logger(), "VisualizeGlobal node shutting down");
}

void VisualizeGlobal::load_raceline_waypoints(const std::string& csv_path) {
    std::ifstream file(csv_path);
    std::string line;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        
        // 세미콜론으로 분리
        while (std::getline(ss, token, ';')) {
            tokens.push_back(token);
        }
        
        // raceline 형식: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
        // x_m은 인덱스 1, y_m은 인덱스 2
        if (tokens.size() >= 3) {
            try {
                float x = std::stof(tokens[1]);
                float y = std::stof(tokens[2]);
                global_waypoints_.push_back({x, y});
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu raceline waypoints", global_waypoints_.size());
}

void VisualizeGlobal::publish_global_waypoints_marker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "global_raceline";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    // raceline은 초록색으로 표시
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for (const auto& wp : global_waypoints_) {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.05;
        marker.points.push_back(p);
    }

    marker_pub_->publish(marker);
}

// 공통 유틸: 섹션/키에서 원문 문자열 얻기
static bool ini_get_raw(const std::string& section, const std::string& key,
                        std::string& out, const std::string& file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) return false;

    std::string line, current;
    bool in_target = false;
    while (std::getline(file, line)) {
        // trim
        auto ltrim = [](std::string& s){ s.erase(0, s.find_first_not_of(" \t")); };
        auto rtrim = [](std::string& s){ s.erase(s.find_last_not_of(" \t")+1); };
        ltrim(line); rtrim(line);
        if (line.empty() || line[0]=='#' || line[0]==';') continue;

        if (line.front()=='[' && line.back()==']') {
            current = line.substr(1, line.size()-2);
            in_target = (current == section);
            continue;
        }
        if (!in_target) continue;

        auto pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos);
        std::string v = line.substr(pos+1);
        ltrim(k); rtrim(k); ltrim(v); rtrim(v);
        if (k == key) { out = v; return true; }
    }
    return false;
}

std::string VisualizeGlobal::ini_load_string(const std::string& section, const std::string& key, const std::string& def, const std::string& file_path)
{
    std::string s;
    if (!ini_get_raw(section, key, s, file_path)) return def;
    return s;
}