#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

// Waypoint 구조체 정의
struct Waypoint {
    float x;
    float y;
};

class VisualizeGlobal : public rclcpp::Node
{
public:
    VisualizeGlobal() : Node("visualize_global")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("global_waypoints_marker", 1);

        // raceline.csv 경로로 변경
        std::string raceline_csv_path = "/home/yongwoo/Raceline-Optimization/outputs/Spielberg_map/traj_race_custom.csv";
        load_raceline_waypoints(raceline_csv_path);

        // 타이머로 주기적으로 marker publish (예: 0.5초마다)
        marker_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VisualizeGlobal::publish_global_waypoints_marker, this)
        );

        RCLCPP_INFO(this->get_logger(), "VisualizeGlobal node initialized with raceline data");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<Waypoint> global_waypoints_;

    void load_raceline_waypoints(const std::string& csv_path) {
        std::ifstream file(csv_path);
        std::string line;
        
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
            return;
        }
        
        // 첫 번째 헤더 라인 건너뛰기
        if (std::getline(file, line)) {
            // 헤더 라인 건너뛰기
        }
        
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            
            // 쉼표로 분리 (QP CSV 파일은 쉼표로 구분됨)
            while (std::getline(ss, token, ',')) {
                tokens.push_back(token);
            }
            
            // QP CSV 형식: x_m, y_m (2개 컬럼만)
            // x_m은 인덱스 0, y_m은 인덱스 1
            if (tokens.size() >= 6) {
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
        file.close();
    }

    void publish_global_waypoints_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "global_raceline";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        // raceline은 파란색으로 표시
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizeGlobal>());
    rclcpp::shutdown();
    return 0;
}