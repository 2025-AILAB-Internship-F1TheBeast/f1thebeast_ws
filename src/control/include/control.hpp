#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <chrono>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// Waypoint 구조체 정의 (기본)
struct Waypoint {
    float x;
    float y;
};

// RacelineWaypoint 구조체 정의 (raceline용)
struct RacelineWaypoint {
    float s;      // 누적 거리 (s_m)
    float x;      // x 좌표 (x_m)
    float y;      // y 좌표 (y_m)
    float psi;    // 헤딩각 (psi_rad)
    float kappa;  // 곡률 (kappa_radpm)
    float vx;     // 목표 속도 (vx_mps)
};

// Evaluation Metrics 구조체
struct EvaluationMetrics {
    double timestamp;
    float cross_track_error;
    float yaw_error;
    float speed_error;
    float current_x;
    float current_y;
    float current_yaw;
    float current_speed;
    float target_speed;
    size_t closest_waypoint_idx;
};

class Control : public rclcpp::Node {
public:
    rclcpp::TimerBase::SharedPtr marker_timer_;

    Control(float stanley_gain = 13.0f);  // 기본값 13.0으로 설정
    ~Control();

private:
    // ROS 관련 멤버
    std::string odom_topic;
    std::string drive_topic;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_waypoints_marker_pub_;

    // PID 관련 멤버 변수
    float pid_integral_;
    float pid_prev_error_;

    // Stanley 제어기 gain
    float stanley_gain_;

    // 전역 변수: 로드된 raceline 웨이포인트
    std::vector<RacelineWaypoint> global_raceline_waypoints_;

    // Evaluation Metrics 관련
    std::vector<EvaluationMetrics> metrics_data_;
    std::ofstream metrics_file_;
    std::chrono::steady_clock::time_point start_time_;

    // 제어 알고리즘 함수들
    float pure_pursuit(float steer_ang_rad, float lookahead_dist);
    float stanley(float car_velocity, std::vector<std::pair<float, float>>& waypoints);
    float point_to_line_distance(float x1, float y1, float x2, float y2, float px, float py);
    std::pair<float, float> vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints, size_t closest_idx);
    float pid_controller(float target_speed, float current_speed);

    // 콜백 및 유틸리티 함수들
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void load_raceline_waypoints(const std::string& csv_path);
    void publish_lookahead_waypoints_marker(const std::vector<RacelineWaypoint>& lookahead_waypoints);
    std::vector<std::pair<float, float>> global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints);

    // Evaluation Metrics 관련 함수들
    void initialize_metrics_csv();
    float calculate_cross_track_error(float car_x, float car_y, size_t closest_idx);
    float calculate_yaw_error(float car_yaw, size_t closest_idx);
    void record_metrics(float car_x, float car_y, float car_yaw, float car_speed, size_t closest_idx, float target_speed);
    void save_metrics_to_csv();
};

#endif // CONTROL_HPP