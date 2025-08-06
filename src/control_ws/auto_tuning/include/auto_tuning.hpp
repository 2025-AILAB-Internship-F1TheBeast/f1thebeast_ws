#ifndef AUTO_TUNING_HPP
#define AUTO_TUNING_HPP

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"  // 추가
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
#include <custom_msgs/msg/wall_collision.hpp>
#include <chrono>
#include <iomanip>  // std::put_time 추가
#include <filesystem>  // std::filesystem 추가

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846  // M_PI 추가
#endif

// RacelineWaypoint 구조체 정의 (raceline용)
struct RacelineWaypoint {
    float s;      // 누적 거리 (s_m)
    float x;      // x 좌표 (x_m)
    float y;      // y 좌표 (y_m)
    float psi;    // 헤딩각 (psi_rad)
    float kappa;  // 곡률 (kappa_radpm)
    float vx;     // 목표 속도 (vx_mps)
    float ax;     // 목표 가속도 (ax_mps2) - 추가
};

// LocalWaypoint 구조체 정의 (로컬 좌표계용)
struct LocalWaypoint {
    float x;
    float y;
    float heading;
    
    LocalWaypoint(float x_, float y_, float heading_) : x(x_), y(y_), heading(heading_) {}
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

class Auto_tuning : public rclcpp::Node {
public:
    rclcpp::TimerBase::SharedPtr marker_timer_;

    Auto_tuning(float stanley_gain_start = 1.0f, int lookahead_idx_start = 0, float stanley_gain_end = 5.0f, float stanley_gain_step = 0.1f, float max_cte_threshold = 2.0f, int max_lookahead_heading = 9);  // max_lookahead_heading 추가
    ~Auto_tuning();

private:
    // ROS 관련 멤버
    std::string odom_topic;
    std::string drive_topic;
    rclcpp::Subscription<custom_msgs::msg::WallCollision>::SharedPtr wall_collision_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_odom_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_waypoints_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr race_reset_publisher_;  // 추가
    
    // 시간 관련 멤버 변수
    rclcpp::Time previous_time_ns;

    // PID 관련 멤버 변수
    float pid_integral_;
    float pid_prev_error_;

    // Stanley 제어기 gain
    float stanley_gain_;
    float stanley_gain_end_;  // 최대 stanley gain 값
    float stanley_gain_step_; // stanley gain 증가량 추가
    
    // Lookahead 인덱스
    int lookahead_heading_;
    int max_lookahead_heading_;  // 최대 lookahead_heading 값 추가
    
    // Waypoint 추적 관련
    size_t current_closest_idx_;
    double previous_velocity_;  // 이전 속도 저장용
    float previous_waypoint_heading = 0.0f;

    // 이전 스티어링 각도 저장용
    float pre_steering_angle = 0.0f;
    float pre_pre_steering_angle = 0.0f;

    // 전역 변수: 로드된 raceline 웨이포인트
    std::vector<RacelineWaypoint> global_raceline_waypoints_;

    // 벽과 충돌했는지 여부를 나타내는 boolean 변수
    bool wall_collision_ = false;

    // Evaluation Metrics 관련
    std::vector<EvaluationMetrics> metrics_data_;
    std::ofstream metrics_file_;
    std::chrono::steady_clock::time_point start_time_;

    // Max 값 추적을 위한 멤버 변수 추가
    float max_cross_track_error_;
    float max_yaw_error_;
    float max_speed_error_;
    float max_cte_threshold_;  // CTE 임계값 추가
    
    // CTE 초과 복구 관련 변수 추가
    rclcpp::Time cte_exceeded_time_;
    bool cte_exceeded_recovery_mode_;
    static constexpr double CTE_EXCEEDED_RECOVERY_DURATION = 0.2; // 0.2초

    // 충돌 복구 관련 변수 추가
    rclcpp::Time collision_time_;
    bool collision_recovery_mode_;
    static constexpr double COLLISION_RECOVERY_DURATION = 0.2; // 0.2초

    // 완주 복구 관련 변수 추가
    rclcpp::Time lap_completion_time_;
    bool lap_completion_recovery_mode_;
    static constexpr double LAP_COMPLETION_RECOVERY_DURATION = 0.2; // 0.2초

    // CSV 파일 관리 관련 변수 추가
    std::string current_csv_filename_;  // 현재 CSV 파일명 저장
    
    // 랩 상태 관리를 위한 멤버 변수들 추가
    size_t initial_closest_idx_;
    bool lap_started_;
    size_t lap_progress_counter_;
    
    // static 변수들을 초기화하는 함수
    void reset_lap_state();

    // 제어 알고리즘 함수들
    float pure_pursuit(float steer_ang_rad, float lookahead_dist);
    float local_planner_based_stanley_controller(float car_velocity, std::vector<LocalWaypoint>& waypoints);
    float stanley_controller(float car_velocity, std::vector<LocalWaypoint>& waypoints);
    float point_to_line_distance_with_heading(float line_x, float line_y, float line_heading, float point_x, float point_y);
    std::pair<float, float> vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints, size_t closest_idx);
    float pid_controller(float target_speed, float current_speed);

    // 콜백 및 유틸리티 함수들
    void wall_collision_callback(const custom_msgs::msg::WallCollision::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void initial_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    size_t find_closest_waypoint_local_search(float global_current_x, float global_current_y);
    void load_raceline_waypoints(const std::string& csv_path);
    void publish_lookahead_waypoints_marker(const std::vector<RacelineWaypoint>& lookahead_waypoints);
    std::vector<LocalWaypoint> global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints);

    // Evaluation Metrics 관련 함수들
    void initialize_metrics_csv();
    float calculate_cross_track_error(float car_x, float car_y, size_t closest_idx);
    float calculate_yaw_error(float car_yaw, size_t closest_idx);
    void record_metrics(float car_x, float car_y, float car_yaw, float car_speed, size_t closest_idx, float target_speed);
    void save_metrics_to_csv();

    // CSV 파일 관리 함수들 추가
    void create_new_csv_file();
    void finalize_current_csv();
    void delete_current_csv();  // 새로 추가
    bool check_lap_completion(size_t current_idx);
};

#endif // AUTO_TUNING_HPP