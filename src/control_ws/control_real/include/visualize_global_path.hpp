#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iomanip>

// Waypoint 구조체 정의
struct Waypoint {
    float x;
    float y;
};

class VisualizeGlobal : public rclcpp::Node
{
public:
    VisualizeGlobal();
    ~VisualizeGlobal();

    void load_raceline_waypoints(const std::string& csv_path);
    void publish_global_waypoints_marker();
    std::string ini_load_string(const std::string& section, const std::string& key, const std::string& def, const std::string& file_path);

private:
    std::string raceline_csv_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
    std::vector<Waypoint> global_waypoints_;   
};