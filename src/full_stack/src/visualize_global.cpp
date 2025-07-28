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

        // CSV 경로를 상황에 맞게 수정하세요
        std::string centerline_csv_path = "/home/jys/ROS2/f1tenth_sim/src/full_stack/map/Spielberg_centerline.csv";
        load_centerline_waypoints(centerline_csv_path);

        // 타이머로 주기적으로 marker publish (예: 0.5초마다)
        marker_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&VisualizeGlobal::publish_global_waypoints_marker, this)
        );

        RCLCPP_INFO(this->get_logger(), "VisualizeGlobal node initialized");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<Waypoint> global_waypoints_;

    void load_centerline_waypoints(const std::string& csv_path) {
        std::ifstream file(csv_path);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            global_waypoints_.push_back({std::stof(x_str), std::stof(y_str)});
        }
    }

    void publish_global_waypoints_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "global_waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizeGlobal>());
    rclcpp::shutdown();
    return 0;
}