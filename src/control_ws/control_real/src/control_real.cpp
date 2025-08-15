#include "control_real.hpp"
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
    odom_topic_("/odom"),
    drive_topic("/drive"), 
    pid_integral_(0.0f),
    pid_prev_error_(0.0f),
    current_closest_idx_(0),
    previous_velocity_(1.0f),
    pid_previous_time_ns_(this->now()),
    lpbsc_previous_time_ns_(this->now()),
    cross_track_error_(0.0f),
    yaw_error_(0.0f),
    ini_file_path_(ini_file_path) {

    // 파라미터 선언
    this->declare_parameter<bool>("use_gt_odom", true);
    this->declare_parameter<bool>("use_pf_odom", false);

    // ② 파라미터 읽기
    const bool use_gt = this->get_parameter("use_gt_odom").as_bool();
    const bool use_pf = this->get_parameter("use_pf_odom").as_bool();

    // ③ 우선순위: GT > PF > 직접지정
    if (use_gt && use_pf) {
        RCLCPP_WARN(this->get_logger(), "Both use_gt_odom and use_pf_odom are true; using GT odom.");
    }
    if (use_gt) {
        odom_topic_ = "/ego_racecar/odom";
    } else if (use_pf) {
        odom_topic_ = "/pf/pose/odom";
    }

    std::string local_path_sub_topic_ = "/chosen_path";

    // target velocity publisher 초기화
    target_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
    car_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("car_velocity", 10);
    cross_track_error_pub_ = this->create_publisher<std_msgs::msg::Float32>("cross_track_error", 10);
    yaw_error_pub_ = this->create_publisher<std_msgs::msg::Float32>("yaw_error", 10);

    // publisher 초기화  
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    text_visualize_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("text_visualize_marker", 1);

    // subscriber 초기화
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&Control::odom_callback, this, std::placeholders::_1));
    
    // local path subscriber 초기화
    local_path_subscription_ = this->create_subscription<custom_msgs::msg::PathPointArray>(
        local_path_sub_topic_, 10, std::bind(&Control::local_path_callback, this, std::placeholders::_1));

    // 제어 타이머 초기화 (예: 50Hz)
    double control_frequency = ini_load_float("control", "control_frequency", 50.0, ini_file_path_);
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency);
    control_timer_ = this->create_wall_timer(
        timer_period, std::bind(&Control::control_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Control timer initialized with frequency: %.1f Hz", control_frequency);
}

// complete : control_class.cpp
Control::~Control() {
    std::cout << "Shutting down Control node..." << std::endl;
}

void Control::local_path_callback(const custom_msgs::msg::PathPointArray::SharedPtr path_msg) {
    std::lock_guard<std::mutex> lock(local_path_mutex_);
    
    // 로컬 패스를 RacelineWaypoint 형태로 변환하여 저장
    current_local_path_.clear();
    
    for (const auto& point : path_msg->points) {
        RacelineWaypoint wp;
        wp.x = point.x;
        wp.y = point.y;
        wp.psi = point.yaw;
        wp.vx = point.v;
        current_local_path_.push_back(wp);
    }
    
    local_path_valid_ = !current_local_path_.empty();
    
    RCLCPP_DEBUG(this->get_logger(), "Local path updated with %zu waypoints", current_local_path_.size());
}

void Control::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    
    // 차량 상태 업데이트
    current_vehicle_state_.x = odom_msg->pose.pose.position.x;
    current_vehicle_state_.y = odom_msg->pose.pose.position.y;
    current_vehicle_state_.yaw = tf2::getYaw(odom_msg->pose.pose.orientation);
    current_vehicle_state_.speed = std::sqrt(
        odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
        odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);
    current_vehicle_state_.timestamp = this->now();
    current_vehicle_state_.valid = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Vehicle state updated: x=%.2f, y=%.2f, yaw=%.2f, speed=%.2f", 
                 current_vehicle_state_.x, current_vehicle_state_.y, 
                 current_vehicle_state_.yaw, current_vehicle_state_.speed);
}

void Control::control_timer_callback() {
    try {
        // 차량 상태와 로컬 패스를 복사 (뮤텍스 보호)
        VehicleState vehicle_state;
        std::vector<RacelineWaypoint> local_path;
        bool path_valid = false;
        
        {
            std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
            vehicle_state = current_vehicle_state_;
        }
        
        {
            std::lock_guard<std::mutex> path_lock(local_path_mutex_);
            local_path = current_local_path_;
            path_valid = local_path_valid_;
        }
        
        // 유효성 검사
        if (!vehicle_state.valid) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Vehicle state not available yet");
            return;
        }
        
        if (!path_valid || local_path.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Local path not available yet");
            return;
        }
        
        // 데이터가 너무 오래된 경우 체크 (예: 1초 이상)
        auto time_diff = (this->now() - vehicle_state.timestamp).seconds();
        if (time_diff > 1.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Vehicle state is too old: %.2f seconds", time_diff);
            return;
        }
        
        // 제어 계산
        auto [steering_angle, drive_speed] = vehicle_control(
            vehicle_state.x, vehicle_state.y, vehicle_state.yaw, 
            vehicle_state.speed, local_path);
        
        // 최소 속도 설정
        drive_speed = std::max(drive_speed, ini_load_float("control", "min_speed", 0.0f, ini_file_path_));
        
        // 제어 명령 퍼블리시
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "ego_racecar/base_link";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;
        
        float control_mode = ini_load_float("control", "control_mode", 0, ini_file_path_);
        if (control_mode == 1.0) {  // Drive mode 0 : Manual control, 1 : Auto control
            drive_pub_->publish(drive_msg);
            RCLCPP_DEBUG(this->get_logger(), 
                        "Published drive command: steering=%.3f rad (%.1f deg), speed=%.2f m/s", 
                        steering_angle, steering_angle * 180.0 / M_PI, drive_speed);
        }
        
        // 디버그 정보 퍼블리시
        std_msgs::msg::Float32 target_velocity_msg;
        target_velocity_msg.data = drive_speed;
        target_velocity_pub_->publish(target_velocity_msg);

        std_msgs::msg::Float32 car_velocity_msg;
        car_velocity_msg.data = vehicle_state.speed;
        car_velocity_pub_->publish(car_velocity_msg);

        std_msgs::msg::Float32 cte_msg;
        cte_msg.data = cross_track_error_;
        cross_track_error_pub_->publish(cte_msg);

        std_msgs::msg::Float32 yaw_error_msg;
        yaw_error_msg.data = yaw_error_;
        yaw_error_pub_->publish(yaw_error_msg);
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during control_timer_callback: %s", e.what());
    }
}

// Pure Pursuit 컨트롤러 수정 및 완성
float Control::pure_pursuit_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {
    publish_car_speed_ = car_speed;  // 퍼블리시할 차량 속도

    // Pure Pursuit 파라미터 로드
    float lookahead_distance = ini_load_float("pure-pursuit", "lookahead_distance", 1.0f, ini_file_path_);
    const float wheelbase = ini_load_float("vehicle", "wheelbase", 0.32f, ini_file_path_);
    publish_lookahead_distance_ = lookahead_distance;  // 퍼블리시할 lookahead distance

    // local waypoint의 좌표를 global 좌표계에서 local 좌표계(base_link 기준)로 변환
    std::vector<LocalWaypoint> local_waypoints = global_to_local(base_link_x, base_link_y, base_link_yaw, waypoints);

    // Lookahead point 찾기
    float best_distance = std::numeric_limits<float>::max();
    size_t target_idx = 0;
    float target_x = 0.0f, target_y = 0.0f;
    bool found_target = false;

    // 먼저 lookahead distance와 가장 가까운 거리에 있는 점을 찾기
    for (size_t i = 0; i < local_waypoints.size(); i++) {
        float distance = std::sqrt(local_waypoints[i].x * local_waypoints[i].x + 
                                 local_waypoints[i].y * local_waypoints[i].y);
        
        // lookahead distance보다 크거나 같고, 차량 앞쪽에 있는 점 중에서 가장 가까운 점
        if (distance >= lookahead_distance && local_waypoints[i].x > 0) {
            if (std::abs(distance - lookahead_distance) < best_distance) {
                best_distance = std::abs(distance - lookahead_distance);
                target_idx = i;
                target_x = local_waypoints[i].x;
                target_y = local_waypoints[i].y;
                found_target = true;
            }
        }
    }

    // 적절한 target이 없으면 가장 먼 앞쪽 점 사용
    if (!found_target) {
        float max_x = -std::numeric_limits<float>::max();
        for (size_t i = 0; i < local_waypoints.size(); i++) {
            if (local_waypoints[i].x > max_x) {
                max_x = local_waypoints[i].x;
                target_idx = i;
                target_x = local_waypoints[i].x;
                target_y = local_waypoints[i].y;
                found_target = true;
            }
        }
    }

    if (!found_target) {
        RCLCPP_WARN(this->get_logger(), "No valid target point found for Pure Pursuit");
        return 0.0f;
    }

    // Target point 시각화
    std::vector<RacelineWaypoint> target_waypoints;
    RacelineWaypoint target_wp;
    target_wp.x = target_x;
    target_wp.y = target_y;
    target_wp.psi = 0.0f;
    target_waypoints.push_back(target_wp);

    // Pure Pursuit 알고리즘
    float lookahead_angle = std::atan2(target_y, target_x);
    float lookahead_dist = std::sqrt(target_x * target_x + target_y * target_y);
    
    // Steering angle 계산
    float steering_angle = std::atan2(2.0f * wheelbase * std::sin(lookahead_angle), lookahead_dist);

    // Cross track error 계산 (디버깅용)
    float cross_track_error = point_to_line_distance_with_heading(
        local_waypoints[target_idx].x, local_waypoints[target_idx].y, 
        local_waypoints[target_idx].heading, 0.0f, 0.0f);
    
    cross_track_error_ = cross_track_error;  // cross_track_error 저장

    // 부호 결정
    if (target_y < 0) {
        cross_track_error *= -1.0f;
    }
    
    yaw_error_ = PI2PI(local_waypoints[target_idx].heading) * 180.0 / M_PI;  // 헤딩 에러를 degree로 변환

    // 디버그 정보 출력
    publishMarker(0.0f, cross_track_error, lookahead_angle, steering_angle);

    return steering_angle;
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

    float dt = (this->now() - pid_previous_time_ns_).seconds();
    pid_previous_time_ns_ = this->now();

    const float stanley_gain = ini_load_float("basic-stanley", "stanley_gain", 0, ini_file_path_);
    const float soft_gain = ini_load_float("basic-stanley", "soft_gain", 0, ini_file_path_);
    const float heading_error_gain = ini_load_float("basic-stanley", "heading_error_gain", 0, ini_file_path_);
    const float cte_pow = ini_load_float("basic-stanley", "cte_pow", 0, ini_file_path_);

    // local waypoint의 좌표를 global 좌표계에서 local 좌표계(base_link 기준)로 변환
    std::vector<LocalWaypoint> local_waypoints = global_to_local(base_link_x, base_link_y, base_link_yaw, waypoints);

    // waypoints가 충분한지 확인
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough waypoints: %zu", waypoints.size());
        return 0.0f;
    }

    // local 좌표계 기준 차량의 앞축의 위치와 waypoints 사이의 거리를 계산하고 거리가 가장 가까운 waypoint 2개의 인덱스 찾기
    float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
    float base_lookahead_dist = ini_load_float("basic-stanley", "lookahead_distance", 0, ini_file_path_);

    // 곡률 기반 lookahead distance 조정 파라미터
    float min_lookahead_dist = ini_load_float("basic-stanley", "min_lookahead_distance", 0, ini_file_path_);
    float max_lookahead_dist = ini_load_float("basic-stanley", "max_lookahead_distance", 0, ini_file_path_);
    float curvature_threshold = ini_load_float("basic-stanley", "curvature_threshold", 0, ini_file_path_);
    bool use_curvature_adaptation = ini_load_bool("basic-stanley", "use_curvature_adaptation", 0, ini_file_path_);

    float lookahead_dist = base_lookahead_dist;

    // waypoint의 곡률을 기반으로 lookahead distance 조정
    if (use_curvature_adaptation && waypoints.size() > 0) {
        // 현재 waypoint의 곡률 값 사용
        float current_kappa = std::abs(waypoints[0].kappa);  // 곡률의 절댓값 사용
        
        // 곡률이 클수록 lookahead distance를 더 작게 조정
        // 곡률이 작을수록 lookahead distance를 더 크게 조정
        if (current_kappa > curvature_threshold) {
            // 높은 곡률 영역: 짧은 lookahead distance
            float curvature_factor = std::min(current_kappa / curvature_threshold, 3.0f);  // 최대 3배까지 조정
            lookahead_dist = std::max(min_lookahead_dist, base_lookahead_dist / curvature_factor);
        } else {
            // 낮은 곡률 영역: 긴 lookahead distance
            float straight_factor = curvature_threshold / std::max(current_kappa, 0.01f);  // 0으로 나누기 방지
            straight_factor = std::min(straight_factor, 3.0f);  // 최대 3배까지 조정
            lookahead_dist = std::min(max_lookahead_dist, base_lookahead_dist * straight_factor);
        }
        
        // // 속도에 따른 추가 조정 (선택사항)
        // bool use_speed_adaptation = ini_load_bool("basic-stanley", "use_speed_adaptation", false, ini_file_path_);
        // if (use_speed_adaptation) {
        //     float speed_factor = std::max(0.5f, std::min(2.0f, car_speed / 3.0f));  // 3m/s 기준으로 조정
        //     lookahead_dist *= speed_factor;
        //     lookahead_dist = std::max(min_lookahead_dist, std::min(max_lookahead_dist, lookahead_dist));
        // }
        
        // 디버그 출력
        std::cout << "Curvature: " << current_kappa << ", Adjusted lookahead: " << lookahead_dist 
                  << " (base: " << base_lookahead_dist << ")" << std::endl;
    }

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

    // // 점과 직선 사이의 거리 계산
    // float dx = local_waypoints[second_idx].x - local_waypoints[first_idx].x;
    // float dy = local_waypoints[second_idx].y - local_waypoints[first_idx].y;
    // float slope = std::atan2(dy, dx);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Option 2 : 차량 앞축과 가장 가까운 점 1개로 cte 계산
    float cross_track_error = point_to_line_distance_with_heading(local_waypoints[first_idx].x, local_waypoints[first_idx].y, local_waypoints[first_idx].heading, front_axle_x, front_axle_y);
    float original_cross_track_error = cross_track_error;  // 원래의 cross_track_error 저장
    cross_track_error_ = original_cross_track_error;  // cross_track_error 저장

    // cross_track_error의 스케일 결정
    // cross_track_error에 제곱승을 적용
    cross_track_error = std::pow(cross_track_error, cte_pow);

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y) / 2 < 0) {
        cross_track_error *= -1.0f;  // 왼쪽에 있으면 음수
        original_cross_track_error *= -1.0;
    }

    // std::cout << "dx: " << dx << ", dy: " << dy << ", slope: " << slope << ", cross_track_error: " << cross_track_error << std::endl;

    // 헤딩 에러 계산
    yaw_error_  = PI2PI(local_waypoints[first_idx].heading) * 180.0 / M_PI;  // 헤딩 에러를 degree로 변환
    float heading_error = heading_error_gain * local_waypoints[first_idx].heading;

    heading_error = PI2PI(heading_error);  // -pi ~ pi 범위로 변환

    // 스티어링 각도 계산
    float cross_track_angle = std::atan2(stanley_gain * cross_track_error, car_speed + soft_gain);

    // 차량의 yaw rate을 보상해줌.
    float k_yaw = ini_load_float("adaptive-stanley", "k_yaw", 0, ini_file_path_);
    float steer_add = -k_yaw * (base_link_yaw - prev_base_link_yaw_) / dt;
    std::cout << "steer_add: " << steer_add * 180 / M_PI << " degrees" << std::endl;

    // curvature based steering angle 계산
    // kappa가 0이 아닌 경우에만 계산 
    float kappa = local_waypoints[0].kappa;  // 곡률
    float m = ini_load_float("adaptive-stanley", "mass", 0, ini_file_path_);
    float C_sr = ini_load_float("adaptive-stanley", "C_sr", 0, ini_file_path_);
    float a = ini_load_float("adaptive-stanley", "lf", 0, ini_file_path_);
    float b = ini_load_float("adaptive-stanley", "lr", 0, ini_file_path_);

    float csi_dot = car_speed * kappa;
    float delta_ssr = m/(C_sr*(1+b/a)) * car_speed * csi_dot;

    float nominator = std::abs(wheelbase * kappa) - std::abs(std::sin(delta_ssr));
    float denominator = std::cos(delta_ssr);
    float steer_kappa = std::atan2(nominator, denominator);


    std::cout << "kappa: " << kappa << std::endl;
    std::cout << "csi_dot: " << csi_dot << std::endl;
    std::cout << "delta_ssr: " << delta_ssr * 180 / M_PI << std::endl;
    std::cout << "nominator: " << nominator << std::endl;
    std::cout << "denominator: " << denominator << std::endl;
    std::cout << "steer_kappa: " << steer_kappa * 180 / M_PI << " degrees" << std::endl;
    float k_steer = ini_load_float("adaptive-stanley", "k_steer", 0, ini_file_path_);

    float steering_angle = heading_error + cross_track_angle + steer_add + k_steer * steer_kappa;
    // std::cout << "After heading_error : " << heading_error * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "cross track angle : " << cross_track_angle * 180 / M_PI << " degrees" << std::endl;
    // std::cout << "steering_angle: " << steering_angle * 180 / M_PI << " degrees" << std::endl;

    publishMarker(heading_error, original_cross_track_error, cross_track_angle, steering_angle);

    prev_base_link_yaw_ = base_link_yaw;  // 이전 base_link_yaw 업데이트

    return steering_angle;
}

float Control::adaptive_stanley_controller(float base_link_x, float base_link_y, float base_link_yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {
    // std::cout << "=====================================================================================" <<  std::endl;
    // Hyperparameter for stanley controller
    publish_car_speed_ = car_speed;  // 퍼블리시할 차량 속도
    const float stanley_gain = ini_load_float("adaptive-stanley", "stanley_gain", 0, ini_file_path_);
    const float soft_gain = ini_load_float("adaptive-stanley", "soft_gain", 0, ini_file_path_);

    // local waypoint의 좌표를 global 좌표계에서 local 좌표계(base_link 기준)로 변환
    std::vector<LocalWaypoint> local_waypoints = global_to_local(base_link_x, base_link_y, base_link_yaw, waypoints);

    // waypoints가 충분한지 확인
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough waypoints: %zu", waypoints.size());
        return 0.0f;
    }

    // local 좌표계 기준 차량의 앞축의 위치와 waypoints 사이의 거리를 계산하고 거리가 가장 가까운 waypoint 2개의 인덱스 찾기
    const float wheelbase = ini_load_float("vehicle", "wheelbase", 0, ini_file_path_);
    float lookahead_dist = 0.0;
    float front_axle_x = 0.0f + wheelbase + lookahead_dist;
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
    float long_lookahead_dist = ini_load_float("adaptive-stanley", "long_lookahead_distance", 0, ini_file_path_);
    float short_lookahead_dist = ini_load_float("adaptive-stanley", "short_lookahead_distance", 0, ini_file_path_);

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y)/2 * local_waypoints[first_idx].kappa > 0) {
        lookahead_dist = long_lookahead_dist;  // 오른쪽에 있으면 긴 lookahead distance
    }
    else {
        lookahead_dist = short_lookahead_dist;  // 왼쪽에 있으면 짧은 lookahead distance
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
    cross_track_error_ = original_cross_track_error;  // cross_track_error 저장

    // cross_track_error의 스케일 결정
    // cross_track_error에 제곱승을 적용
    float cte_pow = ini_load_float("adaptive-stanley", "cte_pow", 0, ini_file_path_);
    cross_track_error = std::pow(cross_track_error, cte_pow);

    // cross_track_error의 부호 결정
    // 가장 가까운 local waypoint 2개의 y의 평균이 0보다 크면 cross_track_error는 양수
    if ((local_waypoints[first_idx].y + local_waypoints[second_idx].y) / 2 < 0) {
        cross_track_error *= -1.0f;  // 왼쪽에 있으면 음수
        original_cross_track_error *= -1.0f;
    }

    // std::cout << "dx: " << dx << ", dy: " << dy << ", slope: " << slope << ", cross_track_error: " << cross_track_error << std::endl;

    // 헤딩 에러 계산
    yaw_error_ = PI2PI(local_waypoints[first_idx].heading) * 180.0 / M_PI;  // 헤딩 에러를 degree로 변환
    float heading_error_gain = ini_load_float("adaptive-stanley", "heading_error_gain", 0, ini_file_path_);
    // std::cout << "heading_error_gain: " << heading_error_gain << std::endl;
    // std::cout << "local_waypoints[first_idx].heading: " << local_waypoints[first_idx].heading << std::endl;
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
    float speed_scale = ini_load_float("control", "speed_scale", 1.0f, ini_file_path_);
    float drive_speed = speed_scale * waypoints[0].vx;  // raceline의 속도 값을 목표 속도로 사용
    float target_speed = drive_speed;  // 속도 스케일링 적용

    // stanley controller 호출해서 스티어링 각도 계산
    float steering_angle = 0.0f;
    bool adaptive_stanley_mode = ini_load_bool("adaptive-stanley", "adaptive_stanley", 0, ini_file_path_);
    bool pure_pursuit_mode = ini_load_bool("pure-pursuit", "pure_pursuit", 0, ini_file_path_);
    RCLCPP_DEBUG(this->get_logger(), "Control mode: adaptive_stanley=%d, pure_pursuit=%d", adaptive_stanley_mode, pure_pursuit_mode);

    if (adaptive_stanley_mode) {
        // Adaptive Stanley Controller 사용
        RCLCPP_DEBUG(this->get_logger(), "Using Adaptive Stanley Controller");
        steering_angle = adaptive_stanley_controller(base_link_x, base_link_y, base_link_yaw, car_speed, waypoints);
    }
    else if (pure_pursuit_mode) {
        // Pure Pursuit Controller 사용
        RCLCPP_DEBUG(this->get_logger(), "Using Pure Pursuit Controller");
        steering_angle = pure_pursuit_controller(base_link_x, base_link_y, base_link_yaw, car_speed, waypoints);
    }
    else {
        // 일반 Stanley Controller 사용
        RCLCPP_DEBUG(this->get_logger(), "Using Stanley Controller");
        steering_angle = stanley_controller(base_link_x, base_link_y, base_link_yaw, car_speed, waypoints);
    }

    return std::make_pair(steering_angle, drive_speed);
}

// complete : control_longitudinal_controller.cpp
float Control::pid_controller(float target_speed, float current_speed) {
    
    float dt = (this->now() - pid_previous_time_ns_).seconds();
    pid_previous_time_ns_ = this->now();

    const float Kp = ini_load_float("pid", "pid_kp", 0, ini_file_path_);
    const float Ki = ini_load_float("pid", "pid_ki", 0, ini_file_path_);
    const float Kd = ini_load_float("pid", "pid_kd", 0, ini_file_path_);

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