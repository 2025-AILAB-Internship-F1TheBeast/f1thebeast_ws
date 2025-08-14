#ifndef CONTROL_REAL_HPP
#define CONTROL_REAL_HPP

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <std_msgs/msg/float32.hpp>
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
#include <custom_msgs/msg/wall_collision.hpp>
#include <custom_msgs/msg/path_point_array.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <iomanip>
#include <mutex>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// RacelineWaypoint 구조체 정의 (raceline용)
struct RacelineWaypoint {
    float s;      // 누적 거리 (s_m)
    float x;      // x 좌표 (x_m)
    float y;      // y 좌표 (y_m)
    float psi;    // 헤딩각 (psi_rad)
    float kappa;  // 곡률 (kappa_radpm)
    float vx;     // 목표 속도 (vx_mps)
};

// LocalWaypoint 구조체 정의 (로컬 좌표계용)
struct LocalWaypoint {
    float x;
    float y;
    float heading;
    float kappa;  // 곡률 (선택적, 필요시 사용)

    LocalWaypoint(float x_, float y_, float heading_, float kappa_ = 0.0f) : x(x_), y(y_), heading(heading_), kappa(kappa_) {}
};

// 차량 상태 구조체
struct VehicleState {
    float x;
    float y;
    float yaw;
    float speed;
    rclcpp::Time timestamp;
    bool valid;

    VehicleState() : x(0.0f), y(0.0f), yaw(0.0f), speed(0.0f), valid(false) {}
};

class Control : public rclcpp::Node {
public:
    Control(std::string ini_file_path);
    ~Control();

    // 실제로 구현된 제어 알고리즘 함수들만 포함
    float pure_pursuit(float steer_ang_rad, float lookahead_dist);
    float pure_pursuit_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints);
    float stanley_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints);
    float adaptive_stanley_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints);
    float point_to_line_distance_with_heading(float line_x, float line_y, float line_heading, float point_x, float point_y);
    std::pair<float, float> vehicle_control(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints);
    float pid_controller(float target_speed, float current_speed);

    // 실제로 구현된 콜백 및 유틸리티 함수들만 포함
    void control_timer_callback();
    void local_path_callback(const custom_msgs::msg::PathPointArray::SharedPtr path_msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    std::vector<LocalWaypoint> global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints);
    void publishMarker(float heading_deg, float cross_track_error, float cross_track_deg, float steering_deg);

    // 실제로 구현된 ini 파일 관련 함수들만 포함
    float ini_load_float(const std::string& section, const std::string& key, float def, const std::string& file_path);
    bool ini_load_bool(const std::string& section, const std::string& key, bool def, const std::string& file_path);
    std::string ini_load_string(const std::string& section, const std::string& key, const std::string& def, const std::string& file_path);

private:
    // ROS 관련 멤버
    std::string odom_topic_;
    std::string drive_topic;
    rclcpp::TimerBase::SharedPtr control_timer_;  // 제어 타이머
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<custom_msgs::msg::PathPointArray>::SharedPtr local_path_subscription_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr car_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cross_track_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_error_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_visualize_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_waypoints_marker_pub_;
    
    // 시간 관련 멤버 변수
    rclcpp::Time pid_previous_time_ns_;
    rclcpp::Time lpbsc_previous_time_ns_;

    // PID 관련 멤버 변수
    float pid_integral_;
    float pid_prev_error_;

    // Waypoint 추적 관련
    size_t current_closest_idx_;
    double previous_velocity_;  // 이전 속도 저장용

    // 이전 base_link yaw 저장용
    float prev_base_link_yaw_ = 30.0 * M_PI / 180.0; // 30도

    // 차 속도 관련 멤버 변수
    float publish_car_speed_ = 0.0f;  // 퍼블리시
    float publish_lookahead_distance_ = 0.0f;  // 퍼블리시

    // Max 값 추적을 위한 멤버 변수 추가
    float cross_track_error_ = 0.0f;
    float yaw_error_ = 0.0f;

    // ini 파일 관련 멤버 변수
    std::string ini_file_path_;

    // 멤버 변수들 - 스레드 안전을 위한 뮤텍스와 함께
    std::mutex vehicle_state_mutex_;
    std::mutex local_path_mutex_;
    VehicleState current_vehicle_state_;
    std::vector<RacelineWaypoint> current_local_path_;
    bool local_path_valid_ = false;
};

#endif // CONTROL_REAL_HPP