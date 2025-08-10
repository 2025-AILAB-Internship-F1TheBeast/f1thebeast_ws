#include "control_sim.hpp"
#include <filesystem>

float PI2PI(float angle) {
    // -pi ~ pi 범위로 변환
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// complete : control_class.cpp
Control::Control(std::string ini_file_path) : 
    Node("controller_node"),
    odom_topic("/odom"),
    drive_topic("/drive"), 
    pid_integral_(0.0f),
    pid_prev_error_(0.0f),
    current_closest_idx_(0),
    previous_velocity_(1.0f),
    pid_previous_time_ns_(this->now()),
    lpbsc_previous_time_ns_(this->now()),
    max_cross_track_error_(0.0f),
    max_yaw_error_(0.0f),
    max_speed_error_(0.0f),
    ini_file_path_(ini_file_path) {

    // target velocity publisher 초기화
    target_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
    car_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("car_velocity", 10);

    // publisher 초기화  
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    lookahead_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_waypoints_marker", 1);
    closest_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("closest_waypoints_marker", 1);
    text_visualize_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("text_visualize_marker", 1);

    // subscriber 초기화
    initial_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&Control::initial_odom_callback, this, std::placeholders::_1));
    
    wall_collision_subscription_ = this->create_subscription<custom_msgs::msg::WallCollision>(
    "/wall_collision", 10, 
    std::bind(&Control::wall_collision_callback, this, std::placeholders::_1));

    // raceline.csv로 경로 변경 /home/jys/ROS2/f1thebeast_ws/src/control_ws/control/map/f1tenth_racetracks
    std::string raceline_csv_path = ini_load_string("paths", "raceline_csv", "", ini_file_path_);
    load_raceline_waypoints(raceline_csv_path);

    // Evaluation metrics 초기화 (enable_metrics가 true일 때만)
    bool enable_metrics_ = ini_load_bool("evaluation", "enable_metrics", 0, ini_file_path_);
    if (enable_metrics_) {
        initialize_metrics_csv();
        start_time_ = std::chrono::steady_clock::now();

    }
}

// complete : control_class.cpp
Control::~Control() {
    std::cout << "Shutting down Control node..." << std::endl;
    // CSV 파일 저장 및 닫기 (metrics가 활성화된 경우에만)
    enable_metrics_ = ini_load_bool("evaluation", "enable_metrics", 0, ini_file_path_);
    if (enable_metrics_) {
        std::cout << "Saving metrics to CSV..." << std::endl;
        save_metrics_to_csv();
        if (metrics_file_.is_open()) {
            metrics_file_.close();
        }
    }
}

// conplete : control_lateral_controller.cpp
float Control::pure_pursuit(float steer_ang_rad, float lookahead_dist) {
    const float lidar_to_rear = 0.27f;
    const float wheel_base = 0.32f;

    float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
    float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);

    float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
    float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + std::pow(bestpoint_y, 2));

    return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
}

float Control::stanley_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {
    // std::cout << "=====================================================================================" <<  std::endl;
    // Hyperparameter for stanley controller
    publish_car_speed_ = car_speed;  // 퍼블리시할 차량 속도
    const float stanley_gain = ini_load_float("control", "stanley_gain", 0, ini_file_path_);
    const float soft_gain = ini_load_float("control", "soft_gain", 0, ini_file_path_);

    // local waypoint의 좌표를 global 좌표계에서 local 좌표계(base_link 기준)로 변환
    std::vector<LocalWaypoint> local_waypoints = global_to_local(base_link_x, base_link_y, base_link_yaw, waypoints);

    // waypoints가 충분한지 확인
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough waypoints: %zu", waypoints.size());
        return 0.0f;
    }

    // local 좌표계 기준 차량의 앞축의 위치와 waypoints 사이의 거리를 계산하고 거리가 가장 가까운 waypoint 2개의 인덱스 찾기
    float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
    float lookahead_dist = ini_load_float("control", "lookahead_distance", 0, ini_file_path_);
    float front_axle_x = 0.0f + wheelbase + lookahead_dist;
    float front_axle_y = 0.0f;
    publish_lookahead_distance_ = lookahead_dist;  // 퍼블리시할 lookahead distance

    // std::cout << "front_axle_x: " << front_axle_x << ", front_axle_y: " << front_axle_y << std::endl;

    float min_dist = std::numeric_limits<float>::max();
    size_t first_idx = 0;
    size_t second_idx = 0;

    // 차량 앞축의 중심점과 가장 가까운 waypoint의 인덱스 찾기
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        float dx = local_waypoints[i].x - front_axle_x;
        float dy = local_waypoints[i].y - front_axle_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            first_idx = i;
        }
    }

    // 차량 앞축의 중심점과 두 번째로 가까운 waypoint의 인덱스 찾기
    min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        if (i == first_idx) continue;  // 첫 번째 waypoint는 건너뜀
        float dx = local_waypoints[i].x - front_axle_x;
        float dy = local_waypoints[i].y - front_axle_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            second_idx = i;
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Option 1 : 차량 앞축과 가장 가까운 점 2개로 cte 계산

    // if (first_idx > second_idx) {
    //     std::swap(first_idx, second_idx);  // 항상 첫 번째 인덱스가 두 번째 인덱스보다 작도록 보장
    // }


    // local_waypoints의 first_idx에 해당하는 정보를 racelinewaypoint와 second_idx를 raceline waypoints의 인덱스로 변환
    // std::vector<RacelineWaypoint> front_axle_closest_waypoints;
    // RacelineWaypoint wp1;
    // wp1.x   = local_waypoints[first_idx].x;
    // wp1.y   = local_waypoints[first_idx].y;
    // wp1.psi = local_waypoints[first_idx].heading;
    // front_axle_closest_waypoints.push_back(wp1);

    // RacelineWaypoint wp2;
    // wp2.x   = local_waypoints[second_idx].x;
    // wp2.y   = local_waypoints[second_idx].y;
    // wp2.psi = local_waypoints[second_idx].heading;
    // front_axle_closest_waypoints.push_back(wp2);

    // publish_closest_waypoints_marker(front_axle_closest_waypoints, 1.0f, 0.0f, 0.0f, "ego_racecar/base_link");  // 빨간색으로 표시

    // // 점과 직선 사이의 거리 계산
    // float dx = local_waypoints[second_idx].x - local_waypoints[first_idx].x;
    // float dy = local_waypoints[second_idx].y - local_waypoints[first_idx].y;
    // float slope = std::atan2(dy, dx);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Option 2 : 차량 앞축과 가장 가까운 점 1개로 cte 계산
    float cross_track_error = point_to_line_distance_with_heading(local_waypoints[first_idx].x, local_waypoints[first_idx].y, local_waypoints[first_idx].heading, front_axle_x, front_axle_y);
    float original_cross_track_error = cross_track_error;  // 원래의 cross_track_error 저장

    // cross_track_error의 스케일 결정
    // cross_track_error에 제곱승을 적용
    float cte_pow = ini_load_float("control", "cte_pow", 0, ini_file_path_);
    cross_track_error = std::pow(cross_track_error, cte_pow);

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y) / 2 < 0) {
        cross_track_error *= -1.0f;  // 왼쪽에 있으면 음수
    }

    // std::cout << "dx: " << dx << ", dy: " << dy << ", slope: " << slope << ", cross_track_error: " << cross_track_error << std::endl;

    // 헤딩 에러 계산
    float heading_error_gain = ini_load_float("control", "heading_error_gain", 0, ini_file_path_);
    float heading_error = heading_error_gain * local_waypoints[first_idx].heading;;

    heading_error = PI2PI(heading_error);  // -pi ~ pi 범위로 변환

    // 스티어링 각도 계산
    float cross_track_angle = std::atan2(stanley_gain * cross_track_error, car_speed + soft_gain);
    float steering_angle = heading_error + cross_track_angle;
    // std::cout << "After heading_error : " << heading_error * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "cross track angle : " << cross_track_angle * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "steering_angle: " << steering_angle * 180 / M_PI << " degrees" << std::endl;

    publishMarker(heading_error, original_cross_track_error, cross_track_angle, steering_angle);

    return steering_angle;
}

float Control::adaptive_stanley_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {
    // std::cout << "=====================================================================================" <<  std::endl;
    // Hyperparameter for stanley controller
    publish_car_speed_ = car_speed;  // 퍼블리시할 차량 속도
    const float stanley_gain = ini_load_float("control", "stanley_gain", 0, ini_file_path_);
    const float soft_gain = ini_load_float("control", "soft_gain", 0, ini_file_path_);

    // local waypoint의 좌표를 global 좌표계에서 local 좌표계(base_link 기준)로 변환
    std::vector<LocalWaypoint> local_waypoints = global_to_local(base_link_x, base_link_y, base_link_yaw, waypoints);

    // waypoints가 충분한지 확인
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough waypoints: %zu", waypoints.size());
        return 0.0f;
    }

    // local 좌표계 기준 차량의 앞축의 위치와 waypoints 사이의 거리를 계산하고 거리가 가장 가까운 waypoint 2개의 인덱스 찾기
    const float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
    float lookahead_dist = ini_load_float("control", "lookahead_distance", 0, ini_file_path_);
    float front_axle_x = 0.0f + wheelbase;
    float front_axle_y = 0.0f;

    float min_dist = std::numeric_limits<float>::max();
    size_t first_idx = 0;
    size_t second_idx = 0;

    // 차량 앞축의 중심점과 가장 가까운 waypoint의 인덱스 찾기
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        float dx = local_waypoints[i].x - front_axle_x;
        float dy = local_waypoints[i].y - front_axle_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            first_idx = i;
        }
    }

    // 차량 앞축의 중심점과 두 번째로 가까운 waypoint의 인덱스 찾기
    min_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        if (i == first_idx) continue;  // 첫 번째 waypoint는 건너뜀
        float dx = local_waypoints[i].x - front_axle_x;
        float dy = local_waypoints[i].y - front_axle_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            second_idx = i;
        }
    }

    float cross_track_error = point_to_line_distance_with_heading(local_waypoints[first_idx].x, local_waypoints[first_idx].y, local_waypoints[first_idx].heading, front_axle_x, front_axle_y);
    float original_cross_track_error = cross_track_error;  // 원래의 cross_track_error 저장

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y)/2 * local_waypoints[first_idx].kappa > 0) {
        lookahead_dist = 0.5;
    }
    else {
        lookahead_dist = 0.1;
    }
    
    front_axle_x = 0.0f + wheelbase + lookahead_dist;
    front_axle_y = 0.0f;
    publish_lookahead_distance_ = lookahead_dist;  // 퍼블리시할 lookahead distance

    // std::cout << "front_axle_x: " << front_axle_x << ", front_axle_y: " << front_axle_y << std::endl;

    min_dist = std::numeric_limits<float>::max();
    first_idx = 0;

    // 차량 앞축의 중심점과 가장 가까운 waypoint의 인덱스 찾기
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        float dx = local_waypoints[i].x - front_axle_x;
        float dy = local_waypoints[i].y - front_axle_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) {
            min_dist = dist;
            first_idx = i;
        }
    }

    cross_track_error = point_to_line_distance_with_heading(local_waypoints[first_idx].x, local_waypoints[first_idx].y, local_waypoints[first_idx].heading, front_axle_x, front_axle_y);
    original_cross_track_error = cross_track_error;  // 원래의 cross_track_error 저장

    // cross_track_error의 스케일 결정
    // cross_track_error에 제곱승을 적용
    float cte_pow = ini_load_float("control", "cte_pow", 0, ini_file_path_);
    cross_track_error = std::pow(cross_track_error, cte_pow);

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y) / 2 < 0) {
        cross_track_error *= -1.0f;  // 왼쪽에 있으면 음수
    }

    // std::cout << "dx: " << dx << ", dy: " << dy << ", slope: " << slope << ", cross_track_error: " << cross_track_error << std::endl;

    // 헤딩 에러 계산
    float heading_error_gain = ini_load_float("control", "heading_error_gain", 0, ini_file_path_);
    float heading_error = heading_error_gain * local_waypoints[first_idx].heading;;

    heading_error = PI2PI(heading_error);  // -pi ~ pi 범위로 변환

    // 스티어링 각도 계산
    float cross_track_angle = std::atan2(stanley_gain * cross_track_error, car_speed + soft_gain);
    float steering_angle = heading_error + cross_track_angle;
    // std::cout << "After heading_error : " << heading_error * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "cross track angle : " << cross_track_angle * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "steering_angle: " << steering_angle * 180 / M_PI << " degrees" << std::endl;

    publishMarker(heading_error, original_cross_track_error, cross_track_angle, steering_angle);

    return steering_angle;

}

float Control::point_to_line_distance_with_heading(float line_x, float line_y, float line_heading, float point_x, float point_y) {
    // line_heading을 이용해서 직선의 방향벡터 계산
    float line_dx = std::cos(line_heading);
    float line_dy = std::sin(line_heading);
    
    // 점에서 직선 위의 한 점까지의 벡터
    float to_point_x = point_x - line_x;
    float to_point_y = point_y - line_y;
    
    // 외적을 이용한 점과 직선 사이의 거리
    // |v1 × v2| = |v1||v2|sin(θ) = distance * |direction_vector|
    // direction_vector의 크기는 1이므로 거리는 외적의 절댓값
    float distance = std::abs(to_point_x * line_dy - to_point_y * line_dx);
    
    return distance;
}

//complete : control_controller.cpp
std::pair<float, float> Control::vehicle_control(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {

    // raceline의 속도 값을 목표 속도로 사용
    // float target_speed = waypoints[3].vx;
    // float target_speed = ini_load_float("control", "target_velocity", 0, ini_file_path_);
    float speed_scale = ini_load_float("control", "speed_scale", 1.0f, ini_file_path_);
    float target_speed = speed_scale * waypoints[0].vx;  // raceline의 속도 값을 목표 속도로 사용
    float drive_speed = speed_scale * waypoints[0].vx;  // raceline의 속도 값을 목표 속도로 사용
    // std::cout << "current_closest_idx_: " << current_closest_idx_ << std::endl;
    // std::cout << "target_speed: " << target_speed << ", drive_speed: " << drive_speed << std::endl;

    // stanley controller 호출해서 스티어링 각도 계산
    float steering_angle = 0.0f;
    bool adaptive_stanley = ini_load_bool("control", "adaptive_stanley", 0, ini_file_path_);
    if (adaptive_stanley) {
        // Adaptive Stanley Controller 사용
        RCLCPP_INFO(this->get_logger(), "Using Adaptive Stanley Controller");
        steering_angle = adaptive_stanley_controller(base_link_x, base_link_y, base_link_yaw, car_speed, waypoints);
    }
    else {
        // 일반 Stanley Controller 사용
        RCLCPP_INFO(this->get_logger(), "Using Stanley Controller");
        steering_angle = stanley_controller(base_link_x, base_link_y, base_link_yaw, car_speed, waypoints);
    }

    // target_speed와 car_speed를 퍼블리시
    std_msgs::msg::Float32 target_velocity_msg;
    target_velocity_msg.data = target_speed;
    target_velocity_pub_->publish(target_velocity_msg);

    std_msgs::msg::Float32 car_velocity_msg;
    car_velocity_msg.data = car_speed;
    car_velocity_pub_->publish(car_velocity_msg);

    return std::make_pair(steering_angle, drive_speed);
}

// complete : control_longitudinal_controller.cpp
float Control::pid_controller(float target_speed, float current_speed) {
    
    float dt = (this->now() - pid_previous_time_ns_).seconds();
    pid_previous_time_ns_ = this->now();

    const float Kp = ini_load_float("control", "pid_kp", 0, ini_file_path_);
    const float Ki = ini_load_float("control", "pid_ki", 0, ini_file_path_);
    const float Kd = ini_load_float("control", "pid_kd", 0, ini_file_path_);

    float error = target_speed - current_speed;

    float P = Kp * error;
    pid_integral_ += error * dt;
    float I = Ki * pid_integral_;
    
    float derivative = (error - pid_prev_error_) / dt;
    float D = Kd * derivative;

    float output = P + I + D;
    pid_prev_error_ = error;

    return output;
}

// complete : control_callback.cpp
void Control::initial_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    try {
        // callback이 제대로 작동하는지 확인
        RCLCPP_INFO(this->get_logger(), "Initial pose callback triggered");

        // base_link 좌표계의 위치와 자세 추출
        float base_link_x = odom_msg->pose.pose.position.x;
        float base_link_y = odom_msg->pose.pose.position.y;
        float global_current_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);

        // base_link에서 front_wheel로 변환 (wheelbase만큼 앞쪽으로 이동)
        const float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
        float global_current_x = base_link_x + wheelbase * std::cos(global_current_yaw);
        float global_current_y = base_link_y + wheelbase * std::sin(global_current_yaw);
        
        // 전체 raceline waypoints에서 가장 가까운 점 찾기 (초기 설정)
        size_t closest_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < global_raceline_waypoints_.size(); ++i) {
            float dx = global_raceline_waypoints_[i].x - global_current_x;
            float dy = global_raceline_waypoints_[i].y - global_current_y;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        current_closest_idx_ = closest_idx;
        RCLCPP_INFO(this->get_logger(), "Initial closest waypoint found: index %zu, distance %.2f", 
                    current_closest_idx_, min_dist);
        
        // 초기 odometry 구독 해제하고 일반 odometry 구독 시작
        initial_odom_subscription_.reset();
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Control::odom_callback, this, std::placeholders::_1));

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during initial_odom_callback: %s", e.what());
    }
}

// complete : control_util.cpp
size_t Control::find_closest_waypoint_local_search(float global_current_x, float global_current_y) {
    const size_t search_range = 20; // 검색 범위를 늘림
    const size_t total_waypoints = global_raceline_waypoints_.size();
    
    size_t closest_idx = current_closest_idx_;
    float min_dist = std::numeric_limits<float>::max();
    
    // 순환 경로를 고려한 검색
    for (int i = -static_cast<int>(search_range); i <= static_cast<int>(search_range); ++i) {
        // 순환 인덱스 계산 (음수도 처리)
        size_t idx = (current_closest_idx_ + i + total_waypoints) % total_waypoints;
        
        float dx = global_raceline_waypoints_[idx].x - global_current_x;
        float dy = global_raceline_waypoints_[idx].y - global_current_y;
        float dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = idx;
        }
    }
    
    // 디버그 출력 추가
    // if (current_closest_idx_ > total_waypoints - 50) {  // 끝 부분에 가까워지면
    //     RCLCPP_INFO(this->get_logger(), "Near end: current_idx=%zu, new_idx=%zu, total=%zu, dist=%.2f", 
    //                 current_closest_idx_, closest_idx, total_waypoints, min_dist);
    // }
    
    return closest_idx;
}
// complete : control_callback.cpp
void Control::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    try {
        // global 좌표계 기준 base_link의 위치와 자세 추출
        float base_link_x = odom_msg->pose.pose.position.x;
        float base_link_y = odom_msg->pose.pose.position.y;
        float base_link_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);

        // std::cout << "base_link_x: " << base_link_x << ", base_link_y: " << base_link_y << ", base_link_yaw: " << base_link_yaw << std::endl;

        // global 좌표계 기준 차량의 앞축 위치 계산
        const float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
        float front_axle_x = base_link_x + wheelbase * std::cos(base_link_yaw);
        float front_axle_y = base_link_y + wheelbase * std::sin(base_link_yaw);
        float front_axle_yaw = base_link_yaw;

        // 현재 차량 속도 계산
        float car_current_speed = std::sqrt(
            odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
            odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);


        // wall_collision이 발생하면 current_closest_idx_를 0으로 초기화
        if (wall_collision_) {
            RCLCPP_WARN(this->get_logger(), "Wall collision detected, resetting current_closest_idx_ to 0");
            current_closest_idx_ = 0;
            wall_collision_ = false;  // 충돌 상태 초기화
        }

        // 효율적인 가장 가까운 waypoint 찾기 (순환 경로 고려)
        // 차량 앞축의 중심점을 기준으로 가장 가까운 waypoint 찾기
        size_t closest_idx = find_closest_waypoint_local_search(base_link_x, base_link_y);
        current_closest_idx_ = closest_idx; // 현재 인덱스 업데이트
        RCLCPP_INFO(this->get_logger(), "Closest waypoint index: %zu", closest_idx);

        // lookahead waypoints 생성
        // base_link 위치와 가장 가까운 waypoint부터 시작하여 lookahead_waypoint_num개의 lookahead waypoints 생성
        int lookahead_waypoint_num = ini_load_float("control", "lookahead_waypoint_num", 0, ini_file_path_);
        std::vector<RacelineWaypoint> lookahead_waypoints;
        for (int i = 0; i < lookahead_waypoint_num; i++) {
            size_t idx = (i + closest_idx + global_raceline_waypoints_.size()) % global_raceline_waypoints_.size();  // 순환 인덱스
            // std::cout << "lookahead_waypoints[" << i << "] index: " << idx << std::endl;
            lookahead_waypoints.push_back(global_raceline_waypoints_[idx]);
            // std::cout << "lookahead_waypoints[" << i << "] vx: " << lookahead_waypoints[i].vx << std::endl;
        }

        // lookahead waypoints 출력
        publish_lookahead_waypoints_marker(lookahead_waypoints, 0.0f, 1.0f, 0.0f, "map");  // 초록색으로 표시

        // steering angle과 drive speed 계산
        auto [steering_angle, drive_speed] = vehicle_control(base_link_x, base_link_y, base_link_yaw, car_current_speed, lookahead_waypoints);

        float target_speed = global_raceline_waypoints_[closest_idx].vx;

        // Evaluation metrics 기록 (metrics가 활성화된 경우에만)
        enable_metrics_ = ini_load_bool("evaluation", "enable_metrics", 0, ini_file_path_);
        if (enable_metrics_) {
            std::cout << "Recording metrics..." << std::endl;
            record_metrics(front_axle_x, front_axle_y, front_axle_yaw, car_current_speed, closest_idx, target_speed);
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;

        float control_mode = ini_load_float("control", "control_mode", 0, ini_file_path_);
        if (control_mode == 1.0) {  // Drive mode 0 : Manual control, 1 : Auto control
            // std::cout << "Publishing drive message with steering angle: " << steering_angle * 180 / M_PI << " degrees, speed: " << drive_speed << std::endl;
            drive_pub_->publish(drive_msg);
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during odom_callback: %s", e.what());
    }
}

// complete : control_metrics.cpp
void Control::load_raceline_waypoints(const std::string& csv_path) {
    std::ifstream file(csv_path);
    std::string line;
    
    // 첫 번째 헤더 라인 건너뛰기
    if (std::getline(file, line)) {
        // 헤더 라인 건너뛰기
    }
    
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
        if (tokens.size() >= 6) {
            try {
                RacelineWaypoint wp;
                wp.s = std::stof(tokens[0]);      // s_m
                wp.x = std::stof(tokens[1]);      // x_m
                wp.y = std::stof(tokens[2]);      // y_m
                wp.psi = std::stof(tokens[3]);    // psi_rad
                wp.kappa = std::stof(tokens[4]);  // kappa_radpm
                wp.vx = std::stof(tokens[5]);     // vx_mps
                global_raceline_waypoints_.push_back(wp);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded %zu raceline waypoints", global_raceline_waypoints_.size());
}

// complete : control_visualize.cpp
// local point의 waypoint를 시각화하는 함수
void Control::publish_lookahead_waypoints_marker(const std::vector<RacelineWaypoint>& lookahead_waypoints, float r, float g, float b, std::string frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "lookahead_raceline";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;  // 삼각형 대신 구(원) 사용
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.04;  // 원의 직경 (x축)
    marker.scale.y = 0.04;  // 원의 직경 (y축)
    marker.scale.z = 0.04;  // 원의 높이 (z축)
    
    for (size_t i = 0; i < lookahead_waypoints.size(); ++i) {
        const auto& wp = lookahead_waypoints[i];
        
        // 원의 중심점 생성
        geometry_msgs::msg::Point point;
        point.x = wp.x;
        point.y = wp.y;
        point.z = 0.05;  // 약간 위로 올림
        
        marker.points.push_back(point);
        
        // 색상 설정
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0f;
        color.r = r;
        color.g = g;
        color.b = b;

        marker.colors.push_back(color);
    }
    
    lookahead_waypoints_marker_pub_->publish(marker);
}


// front_axle과 가장 가까운 waypoint 2개를 시각화하는 함수
void Control::publish_closest_waypoints_marker(const std::vector<RacelineWaypoint>& closest_waypoints, float r, float g, float b, std::string frame_id) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "closest_raceline";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;  // 삼각형 대신 구(원) 사용
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.04;  // 원의 직경 (x축)
    marker.scale.y = 0.04;  // 원의 직각 (y축)
    marker.scale.z = 0.04;  // 원의 높이 (z축)

    for (size_t i = 0; i < closest_waypoints.size(); ++i) {
        const auto& wp = closest_waypoints[i];

        // 원의 중심점 생성
        geometry_msgs::msg::Point point;
        point.x = wp.x;
        point.y = wp.y;
        point.z = 0.05;  // 약간 위로 올림

        marker.points.push_back(point);

        // 색상 설정
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0f;
        color.r = r;
        color.g = g;
        color.b = b;

        marker.colors.push_back(color);
    }

    closest_waypoints_marker_pub_->publish(marker);
}

// complete : control_util.cpp
// global raceline waypoints를 차량 좌표계로 변환하는 함수
std::vector<LocalWaypoint> Control::global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints) {
    std::vector<LocalWaypoint> local_points;
    float cos_yaw = std::cos(-car_yaw);
    float sin_yaw = std::sin(-car_yaw);

    for (const auto& wp : waypoints) {
        // 위치 변환
        float dx = wp.x - car_x;
        float dy = wp.y - car_y;
        float local_x = dx * cos_yaw - dy * sin_yaw;
        float local_y = dx * sin_yaw + dy * cos_yaw;
        float kappa = wp.kappa;  // kappa는 변환하지 않음

        // 헤딩 변환
        float local_heading = wp.psi - car_yaw;
        while (local_heading > M_PI) local_heading -= 2 * M_PI;
        while (local_heading < -M_PI) local_heading += 2 * M_PI;
        
        local_points.emplace_back(local_x, local_y, local_heading, kappa);
    }
    return local_points;
}

// complete : control_metrics.cpp
void Control::initialize_metrics_csv() {
    // evaluation_metrics 폴더 생성
    std::string metrics_dir = ini_load_string("paths", "metrics_output_dir", "", ini_file_path_);
    std::filesystem::create_directories(metrics_dir);

    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    float stanley_gain = ini_load_float("control", "stanley_gain", 0, ini_file_path_);
    float soft_gain = ini_load_float("control", "soft_gain", 0, ini_file_path_);
    filename << metrics_dir << "/" << stanley_gain << "_" << soft_gain << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    metrics_file_.open(filename.str());
    if (metrics_file_.is_open()) {
    metrics_file_ << "timestamp,"
                  << "cross_track_error,"
                  << "yaw_error,"
                  << "speed_error,"
                  << "max_cross_track_error,"
                  << "max_yaw_error,"
                  << "max_speed_error,"
                  << "current_x,"
                  << "current_y,"
                  << "current_yaw,"
                  << "current_speed,"
                  << "target_speed,"
                  << "closest_waypoint_idx\n";
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create metrics CSV file");
    }
}

float Control::calculate_cross_track_error(float car_x, float car_y, size_t closest_idx) {
    if (closest_idx >= global_raceline_waypoints_.size()) {
        return 0.0f;
    }

    // 현재 waypoint의 정보 사용
    float waypoint_x = global_raceline_waypoints_[closest_idx].x;
    float waypoint_y = global_raceline_waypoints_[closest_idx].y;
    float waypoint_heading = global_raceline_waypoints_[closest_idx].psi;

    // heading을 이용한 점과 직선 사이의 거리 계산
    float cross_track_error = point_to_line_distance_with_heading(waypoint_x, waypoint_y, waypoint_heading, car_x, car_y);
    
    // 부호 결정을 위해 차량이 waypoint 기준으로 어느 쪽에 있는지 판단
    // waypoint에서 차량으로의 벡터
    float to_car_x = car_x - waypoint_x;
    float to_car_y = car_y - waypoint_y;
    
    // waypoint heading에 수직인 벡터 (왼쪽 방향)
    float perpendicular_x = -std::sin(waypoint_heading);
    float perpendicular_y = std::cos(waypoint_heading);
    
    // 내적으로 부호 결정 (양수: 왼쪽, 음수: 오른쪽)
    float dot_product = to_car_x * perpendicular_x + to_car_y * perpendicular_y;
    
    if (dot_product < 0) {
        cross_track_error *= -1.0f;  // 오른쪽에 있으면 음수
    }
    
    return cross_track_error;
}

float Control::calculate_yaw_error(float car_yaw, size_t closest_idx) {
    if (closest_idx >= global_raceline_waypoints_.size()) {
        return 0.0f;
    }

    float target_yaw = global_raceline_waypoints_[closest_idx].psi;
    float yaw_error = car_yaw - target_yaw;

    // Yaw error를 [-π, π] 범위로 정규화
    while (yaw_error > PI) yaw_error -= 2 * PI;
    while (yaw_error < -PI) yaw_error += 2 * PI;

    return yaw_error;
}

// pose_callback에서 호출되는 메트릭 기록 함수
// 차량의 현재 위치, 자세와 가장 가까운 waypoint 인덱스를 기록
// 속도는 없음

void Control::record_metrics(float car_x, float car_y, float car_yaw, float car_speed, size_t closest_idx, float target_speed) {
    // 현재 시간 계산 (시작 시간으로부터의 경과 시간)
    
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
    double elapsed_seconds = elapsed.count() / 1000.0; // 초 단위
    
    EvaluationMetrics metrics;
    metrics.timestamp = elapsed_seconds; // 초 단위

    // 에러 계산
    metrics.cross_track_error = calculate_cross_track_error(car_x, car_y, closest_idx);
    metrics.yaw_error = calculate_yaw_error(car_yaw, closest_idx) * 180.0 / PI; // 라디안에서 도 단위로 변환
    metrics.speed_error = std::abs(car_speed - target_speed);

    // Max 값 업데이트
    max_cross_track_error_ = std::max(max_cross_track_error_, std::abs(metrics.cross_track_error));
    max_yaw_error_ = std::max(max_yaw_error_, std::abs(metrics.yaw_error));
    max_speed_error_ = std::max(max_speed_error_, metrics.speed_error);

    // 현재 상태 저장
    metrics.current_x = car_x;
    metrics.current_y = car_y;
    metrics.current_yaw = car_yaw;
    metrics.closest_waypoint_idx = closest_idx;

    // 메모리에 저장
    metrics_data_.push_back(metrics);

    // === 파일 출력: 헤더 순서와 정확히 일치, 포맷 고정 ===
    if (metrics_file_.is_open()) {
        metrics_file_ << std::fixed << std::setprecision(3)
                      << metrics.timestamp << ","
                      << metrics.cross_track_error << ","
                      << metrics.yaw_error << ","
                      << metrics.speed_error << ","
                      << max_cross_track_error_ << ","
                      << max_yaw_error_ << ","
                      << max_speed_error_ << ","
                      << car_x << ","
                      << car_y << ","
                      << car_yaw << ","
                      << car_speed << ","
                      << target_speed << ","
                      << closest_idx << "\n";
        // 매번 flush는 I/O 비용이 큼. 필요 시 주기적으로만 flush 권장
        // metrics_file_.flush();
    }

    // 로그 출력 (max 값 포함)
    RCLCPP_DEBUG(this->get_logger(),
        "Metrics - CTE: %.3f (Max: %.3f), YE: %.3fdeg (Max: %.3fdeg), SE: %.3f (Max: %.3f)",
        metrics.cross_track_error, max_cross_track_error_,
        metrics.yaw_error,        max_yaw_error_,
        metrics.speed_error,      max_speed_error_);
}

void Control::save_metrics_to_csv() {
    if (!metrics_data_.empty()) {
        // 통계 계산
        float avg_cte = 0.0f, avg_ye = 0.0f, avg_se = 0.0f;
        float rms_cte = 0.0f, rms_ye = 0.0f, rms_se = 0.0f;

        for (const auto& metrics : metrics_data_) {
            avg_cte += std::abs(metrics.cross_track_error);
            avg_ye += std::abs(metrics.yaw_error);

            rms_cte += metrics.cross_track_error * metrics.cross_track_error;
            rms_ye += metrics.yaw_error * metrics.yaw_error;
        }

        size_t n = metrics_data_.size();
        avg_cte /= n; avg_ye /= n; avg_se /= n;
        rms_cte = std::sqrt(rms_cte / n);
        rms_ye = std::sqrt(rms_ye / n);
        rms_se = std::sqrt(rms_se / n);

        // 통계 출력 (최종 max 값 사용)
        RCLCPP_INFO(this->get_logger(), "=== Evaluation Metrics Summary ===");
        RCLCPP_INFO(this->get_logger(), "Cross Track Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_cte, max_cross_track_error_, rms_cte);
        RCLCPP_INFO(this->get_logger(), "Yaw Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_ye, max_yaw_error_, rms_ye);
        RCLCPP_INFO(this->get_logger(), "Total data points: %zu", n);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

float Control::ini_load_float(const std::string& section, const std::string& key,
                              float def, const std::string& file_path)
{
    std::string s;
    if (!ini_get_raw(section, key, s, file_path)) return def;
    try { return std::stof(s); }
    catch (...) {
        RCLCPP_WARN(get_logger(), "Invalid float for [%s]%s: '%s' -> use default %.3f",
                    section.c_str(), key.c_str(), s.c_str(), def);
        return def;
    }
}

bool Control::ini_load_bool(const std::string& section, const std::string& key,
                            bool def, const std::string& file_path)
{
    std::string s;
    if (!ini_get_raw(section, key, s, file_path)) return def;
    std::string t; t.resize(s.size());
    std::transform(s.begin(), s.end(), t.begin(), [](unsigned char c){ return std::tolower(c); });
    if (t=="1" || t=="true" || t=="yes" || t=="on")  return true;
    if (t=="0" || t=="false"|| t=="no"  || t=="off") return false;
    RCLCPP_WARN(get_logger(), "Invalid bool for [%s]%s: '%s' -> use default %s",
                section.c_str(), key.c_str(), s.c_str(), def?"true":"false");
    return def;
}

std::string Control::ini_load_string(const std::string& section, const std::string& key,
                                     const std::string& def, const std::string& file_path)
{
    std::string s;
    if (!ini_get_raw(section, key, s, file_path)) return def;
    return s;
}

void Control::publishMarker(float heading_deg, float cross_track_error, float cross_track_deg, float steering_deg) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego_racecar/base_link";  // base_link 좌표계에서 표시
    marker.header.stamp = this->now();
    marker.ns = "stanley_debug";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // RViz에서 보기 좋게 위치
    marker.pose.position.x = 1.0;
    marker.pose.position.y = -1.0;
    marker.pose.position.z = 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.2;   // 글자 크기
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(2);
    ss << "Heading error: " << heading_deg * 180 / M_PI << " deg\n"
       << "Cross track error: " << cross_track_error << " m\n"
       << "Cross track angle: " << cross_track_deg * 180 / M_PI << " deg\n"
       << "Steering angle: " << steering_deg * 180 / M_PI << " deg\n"
       << "Car speed: " << publish_car_speed_ << " m/s\n"
       << "Lookahead distance: " << publish_lookahead_distance_ << " m\n";
    marker.text = ss.str();

    text_visualize_pub_->publish(marker);
}

void Control::wall_collision_callback(const custom_msgs::msg::WallCollision::SharedPtr msg)
{
    wall_collision_ = msg->collision;
}