#include "auto_tuning.hpp"
#include <filesystem>

Auto_tuning::Auto_tuning(float stanley_gain, int lookahead_heading, float stanley_gain_end, float stanley_gain_step, float max_cte_threshold, int max_lookahead_heading) : Node("auto_tuning_node"), odom_topic("/odom"), drive_topic("/drive"), 
    pid_integral_(0.0f), pid_prev_error_(0.0f), stanley_gain_(stanley_gain), stanley_gain_end_(stanley_gain_end), stanley_gain_step_(stanley_gain_step), 
    lookahead_heading_(lookahead_heading), max_lookahead_heading_(max_lookahead_heading), current_closest_idx_(0), previous_velocity_(1.0f),
    previous_time_ns(this->now()), max_cross_track_error_(0.0f), max_yaw_error_(0.0f), max_speed_error_(0.0f), max_cte_threshold_(max_cte_threshold),
    collision_recovery_mode_(false), lap_completion_recovery_mode_(false), cte_exceeded_recovery_mode_(false),
    initial_closest_idx_(0), lap_started_(false), lap_progress_counter_(0) {  // max_lookahead_heading 초기화
    
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    lookahead_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_waypoints_marker", 1);
    race_reset_publisher_ = this->create_publisher<std_msgs::msg::String>("/race_complete_reset_ego", 10);

    // raceline.csv로 경로 변경
    std::string raceline_csv_path = "/home/jys/ROS2/f1thebeast_ws/src/auto_tuning/map/f1tenth_racetracks/Catalunya/Catalunya_raceline.csv";
    load_raceline_waypoints(raceline_csv_path);

    // Evaluation metrics 초기화
    initialize_metrics_csv();
    start_time_ = std::chrono::steady_clock::now();

    // 벽 충돌 구독
    wall_collision_subscription_ = this->create_subscription<custom_msgs::msg::WallCollision>(
    "/wall_collision", 10, 
    std::bind(&Auto_tuning::wall_collision_callback, this, std::placeholders::_1));

    // 초기 위치 설정을 위한 일회성 odometry 구독
    initial_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&Auto_tuning::initial_odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Auto_tuning node initialized with stanley gain: %.2f (max: %.2f, step: %.2f), lookahead heading: %d (max: %d)", 
                stanley_gain_, stanley_gain_end_, stanley_gain_step_, lookahead_heading_, max_lookahead_heading_);
    RCLCPP_INFO(this->get_logger(), "Max CTE threshold set to: %.2f meters", max_cte_threshold_);
    
    // stanley_gain이 이미 end 값에 도달했는지 확인
    if (stanley_gain_ >= stanley_gain_end_) {
        RCLCPP_WARN(this->get_logger(), "Stanley gain start value (%.2f) is already >= end value (%.2f). Node will exit immediately.", 
                    stanley_gain_, stanley_gain_end_);
        rclcpp::shutdown();
        return;
    }
    
    // stanley_gain_step이 유효한지 확인
    if (stanley_gain_step_ <= 0.0f) {
        RCLCPP_WARN(this->get_logger(), "Stanley gain step (%.2f) must be positive. Setting to default 0.1", stanley_gain_step_);
        stanley_gain_step_ = 0.1f;
    }
    
    // max_cte_threshold가 유효한지 확인
    if (max_cte_threshold_ <= 0.0f) {
        RCLCPP_WARN(this->get_logger(), "Max CTE threshold (%.2f) must be positive. Setting to default 2.0", max_cte_threshold_);
        max_cte_threshold_ = 2.0f;
    }
    
    // max_lookahead_heading이 유효한지 확인
    if (max_lookahead_heading_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Max lookahead heading (%d) must be non-negative. Setting to default 9", max_lookahead_heading_);
        max_lookahead_heading_ = 9;
    }
    
    if (lookahead_heading_ > max_lookahead_heading_) {
        RCLCPP_WARN(this->get_logger(), "Lookahead heading start (%d) is greater than max (%d). Setting to 0", lookahead_heading_, max_lookahead_heading_);
        lookahead_heading_ = 0;
    }
}

Auto_tuning::~Auto_tuning() {
    // CSV 파일 저장 및 닫기
    save_metrics_to_csv();
    if (metrics_file_.is_open()) {
        metrics_file_.close();
    }
}

// 벽 충돌용 콜백 함수 구현
void Auto_tuning::wall_collision_callback(const custom_msgs::msg::WallCollision::SharedPtr msg) {
    wall_collision_ = msg->collision;
    RCLCPP_INFO(this->get_logger(), "Wall collision occurred! %s", wall_collision_ ? "True" : "False");
}

float Auto_tuning::pure_pursuit(float steer_ang_rad, float lookahead_dist) {
    const float lidar_to_rear = 0.27f;
    const float wheel_base = 0.32f;

    float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
    float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);

    float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
    float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + std::pow(bestpoint_y, 2));

    return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
}

float Auto_tuning::local_planner_based_stanley_controller(float car_velocity, std::vector<LocalWaypoint>& waypoints) {
    // Hyperparameter for stanley controller
    const float k_angle_gain = 1.0;
    const float k_cte_gain = 1.3;
    const float k_dist_gain = 5.5;
    const float k_damp_gain = 1.0;
    const float k_soft_gain = 2.0;
    const float k_yaw_rate_gain = 0.006;
    const float k_steer_gain = 0.4;

    const float x_front = 0.0f;
    const float y_front = 0.0f;

    // 현재 시각 (나노초 단위)
    rclcpp::Time current_time_ns = this->now();

    double dt = (current_time_ns - previous_time_ns).seconds();
    previous_time_ns = current_time_ns;

    float waypoint_x = waypoints[lookahead_heading_].x;
    float waypoint_y = waypoints[lookahead_heading_].y;
    float waypoint_heading = waypoints[lookahead_heading_].heading;

    // heading을 이용한 점과 직선 사이의 거리 계산
    float track_error = point_to_line_distance_with_heading(waypoint_x, waypoint_y, waypoint_heading, x_front, y_front);
    std::cout << "트랙과의 거리 : " << track_error << std::endl;
    // 부호 결정 (waypoint가 차량의 왼쪽에 있으면 음수)
    if (waypoint_y < 0.0f) {
        track_error *= -1.0;
    }

    // delta 계산
    float delta_heading_error = waypoint_heading;
    float delta_cte = std::atan2(k_dist_gain * track_error, k_damp_gain * car_velocity + k_soft_gain);
    float delta_yaw_error = (waypoint_heading - previous_waypoint_heading) / dt;
    float delta_steering_angle = pre_steering_angle - pre_pre_steering_angle;

    // delta 값 출력
    std::cout << "delta_heading_error : " << k_angle_gain * delta_heading_error * 180/PI << std::endl;
    std::cout << "delta_cte : " << k_cte_gain * std::atan2(k_dist_gain * delta_cte, car_velocity * k_damp_gain + k_soft_gain) * 180/PI << std::endl;
    std::cout << "delta_yaw_error : " << k_yaw_rate_gain * delta_yaw_error * 180/PI << std::endl;
    std::cout << "delta_steering_angle : " << k_steer_gain * delta_steering_angle * 180/PI << std::endl;

    // 최종 steering angle 계산
    float steering_angle = k_angle_gain * delta_heading_error + k_cte_gain * std::atan2(k_dist_gain * delta_cte, car_velocity * k_damp_gain + k_soft_gain)
                            + k_yaw_rate_gain * delta_yaw_error + k_steer_gain * delta_steering_angle;
    std::cout << "steering_angle : " << steering_angle * 180/PI << std::endl;

    // 이전 값 업데이트
    pre_pre_steering_angle = pre_steering_angle;
    pre_steering_angle = steering_angle;
    previous_waypoint_heading = waypoint_heading;
    return steering_angle;
}

float Auto_tuning::stanley_controller(float car_velocity, std::vector<LocalWaypoint>& waypoints) {
    // Hyperparameter for stanley controller
    const float k_dist_gain = stanley_gain_;
    // const float k_soft_gain = 5.0;

    const float x_front = 0.0f;
    const float y_front = 0.0f;
    
    // lookahead_heading_ 인덱스 사용 (범위 확인)
    float waypoint_x = waypoints[0].x;
    float waypoint_y = waypoints[0].y;
    float waypoint_heading = waypoints[0].heading;

    // lookahead_heading_ 인덱스가 waypoints 범위 내에 있는지 확인
    int heading_idx = std::min(lookahead_heading_, static_cast<int>(waypoints.size()) - 1);
    // waypoint_x = waypoints[heading_idx].x;
    // waypoint_y = waypoints[heading_idx].y;
    waypoint_heading = waypoints[heading_idx].heading;
    float heading_error = waypoint_heading;

    // heading을 이용한 점과 직선 사이의 거리 계산
    float track_error = point_to_line_distance_with_heading(waypoint_x, waypoint_y, waypoint_heading, x_front, y_front);

    // 부호 결정 (waypoint가 차량의 왼쪽에 있으면 음수)
    if (waypoint_y < 0.0f) {
        track_error *= -1.0;
    }

    // steering angle 계산
    float steering_angle = heading_error + std::atan2(k_dist_gain * track_error, car_velocity);

    return steering_angle;
}

// 새로운 point_to_line_distance_with_heading 함수 (heading 사용)
float Auto_tuning::point_to_line_distance_with_heading(float line_x, float line_y, float line_heading, float point_x, float point_y) {
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

std::pair<float, float> Auto_tuning::vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints, size_t closest_idx) {
    
    // std::cout << "============================================" << std::endl;
    // std::cout << "Global Car Position: (" << global_car_x << ", " << global_car_y << "), Yaw: " << yaw << ", Speed: " << car_speed << std::endl;
    // std::cout << "Closest Waypoint Position: (" << waypoints[closest_idx].x << ", " << waypoints[closest_idx].y << ")" << "Yaw: " << waypoints[closest_idx].psi << std::endl;

    // local_path의 좌표를 global 좌표계에서 local 좌표계로 변환
    std::vector<LocalWaypoint> local_points = global_to_local(global_car_x, global_car_y, yaw, waypoints);
    float current_speed = car_speed;

    // local_path의 좌표를 모두 출력
    // for (const auto& point : local_points) {
    //     std::cout << "Local Point: (" << point.first << ", " << point.second << ")" << std::endl;
    // }

    // raceline의 속도 값을 목표 속도로 사용
    float target_speed = waypoints[closest_idx].vx;
    float drive_speed = pid_controller(target_speed, current_speed);
    drive_speed = target_speed;
    // float stanley_steer = local_planner_based_stanley_controller(current_speed, local_points);
    float stanley_steer = stanley_controller(current_speed, local_points);
    // stanley_steer = 0.0;
    // drive_speed = 0.0;
    return std::make_pair(stanley_steer, drive_speed);
}

float Auto_tuning::pid_controller(float target_speed, float current_speed) {
    float dt = 0.1;

    const float Kp = 5.0f;
    const float Ki = 1.5f;
    const float Kd = 4.2f;

    float error = target_speed - current_speed;

    float P = Kp * error;
    pid_integral_ += error * dt;
    float I = Ki * pid_integral_;
    
    float derivative = (error - pid_prev_error_) / dt;
    float D = Kd * derivative;

    float output = P + I + D;
    pid_prev_error_ = error;

    // std::cout << "Target Speed: " << target_speed << ", Current Speed: " << current_speed << ", PID Output: " << output << std::endl;

    return output;
}

void Auto_tuning::initial_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    try {
        // base_link 좌표계의 위치와 자세 추출
        float base_link_x = odom_msg->pose.pose.position.x;
        float base_link_y = odom_msg->pose.pose.position.y;
        float global_current_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);
        
        // base_link에서 front_wheel로 변환 (wheelbase만큼 앞쪽으로 이동)
        const float wheelbase = 0.3302f;
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
            "/ego_racecar/odom", 10, std::bind(&Auto_tuning::odom_callback, this, std::placeholders::_1));
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during initial_odom_callback: %s", e.what());
    }
}

size_t Auto_tuning::find_closest_waypoint_local_search(float global_current_x, float global_current_y) {
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
    
    return closest_idx;
}

void Auto_tuning::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    try {
        // 충돌 복구 모드 확인
        if (collision_recovery_mode_) {
            double elapsed_time = (this->now() - collision_time_).seconds();
            if (elapsed_time < COLLISION_RECOVERY_DURATION) {
                // 복구 중이므로 정지 명령만 전송
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                drive_msg.drive.steering_angle = 0.0;
                drive_msg.drive.speed = 0.0;
                drive_publisher_->publish(drive_msg);
                return; // 조기 리턴
            } else {
                // 복구 완료
                collision_recovery_mode_ = false;
                RCLCPP_INFO(this->get_logger(), "Collision recovery completed (0.5s)");
            }
        }

        // 완주 복구 모드 확인
        if (lap_completion_recovery_mode_) {
            double elapsed_time = (this->now() - lap_completion_time_).seconds();
            if (elapsed_time < LAP_COMPLETION_RECOVERY_DURATION) {
                // 완주 후 복구 중이므로 정지 명령만 전송
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                drive_msg.drive.steering_angle = 0.0;
                drive_msg.drive.speed = 0.0;
                drive_publisher_->publish(drive_msg);
                return; // 조기 리턴
            } else {
                // 완주 복구 완료
                lap_completion_recovery_mode_ = false;
                RCLCPP_INFO(this->get_logger(), "Lap completion recovery completed (0.5s), starting new lap");
            }
        }

        // CTE 초과 복구 모드 확인
        if (cte_exceeded_recovery_mode_) {
            double elapsed_time = (this->now() - cte_exceeded_time_).seconds();
            if (elapsed_time < CTE_EXCEEDED_RECOVERY_DURATION) {
                // CTE 초과 복구 중이므로 정지 명령만 전송
                auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                drive_msg.drive.steering_angle = 0.0;
                drive_msg.drive.speed = 0.0;
                drive_publisher_->publish(drive_msg);
                return; // 조기 리턴
            } else {
                // CTE 초과 복구 완료
                cte_exceeded_recovery_mode_ = false;
                RCLCPP_INFO(this->get_logger(), "CTE exceeded recovery completed (0.5s), starting new lap");
            }
        }

        // gym_bridge에서 벽에 부딪혔다는 신호가 들어오면
        if (wall_collision_) {
            RCLCPP_WARN(this->get_logger(), "Collision detected! Current params - stanley_gain: %.2f, lookahead_heading: %d", 
                        stanley_gain_, lookahead_heading_);
            
            // 현재 CSV 파일 삭제 (충돌로 인한 삭제)
            delete_current_csv();
            
            // 파라미터 업데이트
            lookahead_heading_++;
            
            // 모든 인덱스 및 랩 상태 초기화
            current_closest_idx_ = 0;
            reset_lap_state();
            
            // stanley_gain과 lookahead_heading을 자동적으로 바꾸면서 최적의 파라미터를 찾는 로직을 추가
            if (lookahead_heading_ == max_lookahead_heading_) {  // max_lookahead_heading_ 사용
                float new_stanley_gain = stanley_gain_ + stanley_gain_step_;  // step 사용
                
                // Stanley gain이 최대값에 도달했는지 확인
                if (new_stanley_gain >= stanley_gain_end_) {
                    RCLCPP_INFO(this->get_logger(), "=== AUTO TUNING COMPLETED ===");
                    RCLCPP_INFO(this->get_logger(), "Stanley gain reached maximum value: %.2f (limit: %.2f)", 
                                new_stanley_gain, stanley_gain_end_);
                    RCLCPP_INFO(this->get_logger(), "Auto tuning process finished. Shutting down node...");
                    
                    // 최종 CSV 파일 정리
                    if (metrics_file_.is_open()) {
                        metrics_file_.close();
                    }
                    
                    // 노드 종료
                    rclcpp::shutdown();
                    return;
                }
                
                // stanley_gain 업데이트 (CSV 파일 생성 전에!)
                stanley_gain_ = new_stanley_gain;
                lookahead_heading_ = 0;
                std::cout << "Lookahead heading reached " << (max_lookahead_heading_ + 1) << ", increasing stanley_gain to " << stanley_gain_ << " (step: " << stanley_gain_step_ << ")" << std::endl;
                RCLCPP_INFO(this->get_logger(), "Updated stanley_gain to %.2f (remaining: %.2f, step: %.2f)", 
                            stanley_gain_, stanley_gain_end_ - stanley_gain_, stanley_gain_step_);
            }
            
            // 새로운 CSV 파일 생성 (파라미터 업데이트 후에!)
            create_new_csv_file();
            
            // 충돌 복구 모드 시작
            collision_time_ = this->now();
            collision_recovery_mode_ = true;
            wall_collision_ = false; // 벽 충돌 상태 초기화
            
            // 즉시 정지 명령 전송
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 0.0;
            drive_publisher_->publish(drive_msg);
            
            RCLCPP_INFO(this->get_logger(), "All indices and lap state reset after collision");
            return; // 조기 리턴
        }

        // base_link 좌표계의 위치와 자세 추출
        float base_link_x = odom_msg->pose.pose.position.x;
        float base_link_y = odom_msg->pose.pose.position.y;
        float global_current_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);
        
        // base_link에서 front_wheel로 변환 (wheelbase만큼 앞쪽으로 이동)
        const float wheelbase = 0.3302f;
        float global_current_x = base_link_x + wheelbase * std::cos(global_current_yaw);
        float global_current_y = base_link_y + wheelbase * std::sin(global_current_yaw);
        
        float car_current_speed = std::sqrt(
            odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
            odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);

        // 효율적인 가장 가까운 waypoint 찾기 (순환 경로 고려)
        size_t closest_idx = find_closest_waypoint_local_search(global_current_x, global_current_y);
        current_closest_idx_ = closest_idx; // 현재 인덱스 업데이트
        
        // CTE 확인 (metrics 기록 전에 체크)
        float current_cte = std::abs(calculate_cross_track_error(global_current_x, global_current_y, closest_idx));
        
        // CTE가 임계값을 초과했는지 확인
        if (current_cte >= max_cte_threshold_) {
            // RCLCPP_WARN(this->get_logger(), "=== CTE EXCEEDED THRESHOLD! ===");
            // RCLCPP_WARN(this->get_logger(), "Current CTE: %.3f >= Threshold: %.3f", current_cte, max_cte_threshold_);
            // RCLCPP_WARN(this->get_logger(), "Current params - stanley_gain: %.2f, lookahead_heading: %d", 
            //             stanley_gain_, lookahead_heading_);
            
            // 차량의 위치 알리기
            // std::cout << "CTE exceeded position - x: " << odom_msg->pose.pose.position.x 
            //           << ", y: " << odom_msg->pose.pose.position.y << std::endl;
            // std::cout << "current_closest_idx_: " << current_closest_idx_ << std::endl;
            
            // 시뮬레이터에 리셋 신호 전송
            auto reset_msg = std_msgs::msg::String();
            reset_msg.data = "cte_exceeded";
            race_reset_publisher_->publish(reset_msg);
            // RCLCPP_INFO(this->get_logger(), "Published CTE exceeded reset signal to simulator");
            
            // 현재 CSV 파일 삭제 (CTE 초과로 인한 삭제)
            delete_current_csv();
            
            // 파라미터 업데이트
            lookahead_heading_++;
            
            // 모든 인덱스 및 랩 상태 초기화
            current_closest_idx_ = 0;
            reset_lap_state();
            
            // stanley_gain과 lookahead_heading을 자동적으로 바꾸면서 최적의 파라미터를 찾는 로직을 추가
            if (lookahead_heading_ == max_lookahead_heading_) {  // max_lookahead_heading_ 사용
                float new_stanley_gain = stanley_gain_ + stanley_gain_step_;  // step 사용
                
                // Stanley gain이 최대값에 도달했는지 확인
                if (new_stanley_gain >= stanley_gain_end_) {
                    RCLCPP_INFO(this->get_logger(), "=== AUTO TUNING COMPLETED ===");
                    RCLCPP_INFO(this->get_logger(), "Stanley gain reached maximum value: %.2f (limit: %.2f)", 
                                new_stanley_gain, stanley_gain_end_);
                    RCLCPP_INFO(this->get_logger(), "Auto tuning process finished. Shutting down node...");
                    
                    // 최종 CSV 파일 정리
                    if (metrics_file_.is_open()) {
                        metrics_file_.close();
                    }
                    
                    // 노드 종료
                    rclcpp::shutdown();
                    return;
                }
                
                // stanley_gain 업데이트 (CSV 파일 생성 전에!)
                stanley_gain_ = new_stanley_gain;
                lookahead_heading_ = 0;
                std::cout << "Lookahead heading reached " << (max_lookahead_heading_ + 1) << ", increasing stanley_gain to " << stanley_gain_ << " (step: " << stanley_gain_step_ << ")" << std::endl;
                RCLCPP_INFO(this->get_logger(), "Updated stanley_gain to %.2f (remaining: %.2f, step: %.2f)", 
                            stanley_gain_, stanley_gain_end_ - stanley_gain_, stanley_gain_step_);
            }
            
            // 새로운 CSV 파일 생성 (파라미터 업데이트 후에!)
            create_new_csv_file();
            
            // CTE 초과 복구 모드 시작
            cte_exceeded_time_ = this->now();
            cte_exceeded_recovery_mode_ = true;
            
            // 즉시 정지 명령 전송
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = 0.0;
            drive_msg.drive.speed = 0.0;
            drive_publisher_->publish(drive_msg);
            
            RCLCPP_INFO(this->get_logger(), "Starting CTE exceeded recovery (0.5s). Next attempt params - stanley_gain: %.2f, lookahead_heading: %d", 
                        stanley_gain_, lookahead_heading_);
            RCLCPP_INFO(this->get_logger(), "All indices and lap state reset after CTE exceeded");
            
            return; // 조기 리턴
        }
        
        // 완주 확인 - 멤버 변수 사용
        if (!lap_started_ && metrics_data_.size() > 100) { // 100개 데이터 포인트 이후
            lap_started_ = true;
            RCLCPP_INFO(this->get_logger(), "Lap started from waypoint index: %zu", initial_closest_idx_);
        }
        
        // 완주 확인 (랩이 시작된 후 시작점 근처로 복귀)
        if (lap_started_) {
            size_t total_waypoints = global_raceline_waypoints_.size();
            size_t distance_from_start;
            
            if (closest_idx >= initial_closest_idx_) {
                distance_from_start = closest_idx - initial_closest_idx_;
            } else {
                distance_from_start = (total_waypoints - initial_closest_idx_) + closest_idx;
            }
            
            // 전체 경로의 90% 이상 진행하고 시작점 근처(±30 waypoints)에 도달하면 완주
            if (distance_from_start >= total_waypoints * 0.9) {
                size_t start_range = 30;
                if ((closest_idx <= initial_closest_idx_ + start_range && closest_idx >= initial_closest_idx_) ||
                    (initial_closest_idx_ < start_range && closest_idx >= total_waypoints - (start_range - initial_closest_idx_))) {
                    
                    RCLCPP_INFO(this->get_logger(), "=== LAP COMPLETED! ===");
                    RCLCPP_INFO(this->get_logger(), "Parameters - stanley_gain: %.2f, lookahead_heading: %d", 
                                stanley_gain_, lookahead_heading_);
                    RCLCPP_INFO(this->get_logger(), "Started at waypoint: %zu, finished at waypoint: %zu", 
                                initial_closest_idx_, closest_idx);
                    
                    // 시뮬레이터에 완주 리셋 신호 전송
                    auto reset_msg = std_msgs::msg::String();
                    reset_msg.data = "lap_completed";
                    race_reset_publisher_->publish(reset_msg);
                    RCLCPP_INFO(this->get_logger(), "Published race complete reset signal to simulator");
                    
                    // 현재 CSV 파일 완료 및 저장 (완주는 저장)
                    finalize_current_csv();
                    
                    // 파라미터 업데이트
                    lookahead_heading_++;
                    if (lookahead_heading_ == max_lookahead_heading_) {  // max_lookahead_heading_ 사용
                        float new_stanley_gain = stanley_gain_ + stanley_gain_step_;  // step 사용
                        
                        // Stanley gain이 최대값에 도달했는지 확인
                        if (new_stanley_gain >= stanley_gain_end_) {
                            RCLCPP_INFO(this->get_logger(), "=== AUTO TUNING COMPLETED ===");
                            RCLCPP_INFO(this->get_logger(), "Stanley gain reached maximum value: %.2f (limit: %.2f)", 
                                        new_stanley_gain, stanley_gain_end_);
                            RCLCPP_INFO(this->get_logger(), "Auto tuning process finished. Shutting down node...");
                            
                            // 최종 CSV 파일 정리
                            if (metrics_file_.is_open()) {
                                metrics_file_.close();
                            }
                            
                            // 노드 종료
                            rclcpp::shutdown();
                            return;
                        }
                        
                        // stanley_gain 업데이트 (CSV 파일 생성 전에!)
                        stanley_gain_ = new_stanley_gain;
                        lookahead_heading_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Updated stanley_gain to %.2f (remaining: %.2f, step: %.2f), reset lookahead_heading to 0", 
                                    stanley_gain_, stanley_gain_end_ - stanley_gain_, stanley_gain_step_);
                    }
                    
                    // 새로운 CSV 파일 생성 (파라미터 업데이트 후에!)
                    create_new_csv_file();
                    
                    // 완주 복구 모드 시작
                    lap_completion_time_ = this->now();
                    lap_completion_recovery_mode_ = true;

                    // 모든 인덱스 및 랩 상태 초기화
                    current_closest_idx_ = 0;
                    closest_idx = 0;
                    reset_lap_state();
                    
                    // 즉시 정지 명령 전송
                    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
                    drive_msg.drive.steering_angle = 0.0;
                    drive_msg.drive.speed = 0.0;
                    drive_publisher_->publish(drive_msg);
                    
                    RCLCPP_INFO(this->get_logger(), "Starting lap completion recovery (0.5s). Next lap params - stanley_gain: %.2f, lookahead_heading: %d", 
                                stanley_gain_, lookahead_heading_);
                    RCLCPP_INFO(this->get_logger(), "All indices and lap state reset for new lap");
                    
                    return; // 조기 리턴
                }
            }
        }

        // lookahead waypoints 생성 (순환 경로 고려)
        std::vector<RacelineWaypoint> lookahead_waypoints;
        for (size_t i = 0; i < max_lookahead_heading_+1; ++i) {
            size_t idx = (closest_idx + i) % global_raceline_waypoints_.size();  // 순환 인덱스
            lookahead_waypoints.push_back(global_raceline_waypoints_[idx]);
        }

        publish_lookahead_waypoints_marker(lookahead_waypoints);

        auto [steering_angle, drive_speed] = vehicle_control(global_current_x, global_current_y, global_current_yaw, car_current_speed, lookahead_waypoints, 0);

        // Evaluation metrics 기록
        float target_speed = global_raceline_waypoints_[closest_idx].vx;
        record_metrics(global_current_x, global_current_y, global_current_yaw, car_current_speed, closest_idx, target_speed);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = drive_speed;
        drive_publisher_->publish(drive_msg);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during odom_callback: %s", e.what());
    }
}

void Auto_tuning::reset_lap_state() {
    initial_closest_idx_ = current_closest_idx_;
    lap_started_ = false;
    lap_progress_counter_ = 0;
    RCLCPP_INFO(this->get_logger(), "Lap state reset for new attempt, starting from waypoint: %zu", initial_closest_idx_);
}

void Auto_tuning::load_raceline_waypoints(const std::string& csv_path) {
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

void Auto_tuning::publish_lookahead_waypoints_marker(const std::vector<RacelineWaypoint>& lookahead_waypoints) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "lookahead_raceline";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;  // 초록색으로 변경 (raceline lookahead)
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    const float tri_size = 0.15f;
    for (const auto& wp : lookahead_waypoints) {
        geometry_msgs::msg::Point p1, p2, p3;
        p1.x = wp.x;
        p1.y = wp.y + tri_size;
        p1.z = 0.05;
        p2.x = wp.x - tri_size * 0.866f;
        p2.y = wp.y - tri_size * 0.5f;
        p2.z = 0.05;
        p3.x = wp.x + tri_size * 0.866f;
        p3.y = wp.y - tri_size * 0.5f;
        p3.z = 0.05;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);
    }
    lookahead_waypoints_marker_pub_->publish(marker);
}

std::vector<LocalWaypoint> Auto_tuning::global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints) {
    std::vector<LocalWaypoint> local_points;
    float cos_yaw = std::cos(-car_yaw);
    float sin_yaw = std::sin(-car_yaw);

    for (const auto& wp : waypoints) {
        // 위치 변환
        float dx = wp.x - car_x;
        float dy = wp.y - car_y;
        float local_x = dx * cos_yaw - dy * sin_yaw;
        float local_y = dx * sin_yaw + dy * cos_yaw;
        
        // 헤딩 변환
        float local_heading = wp.psi - car_yaw;
        while (local_heading > M_PI) local_heading -= 2 * M_PI;
        while (local_heading < -M_PI) local_heading += 2 * M_PI;
        
        local_points.emplace_back(local_x, local_y, local_heading);
    }
    return local_points;
}

void Auto_tuning::initialize_metrics_csv() {
    // evaluation_metrics 폴더 생성
    std::string metrics_dir = "/home/jys/ROS2/f1thebeast_ws/src/auto_tuning/evaluation_metrics/csv_file/Catalunya";
    std::filesystem::create_directories(metrics_dir);

    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);

    // 파일명 생성
    std::stringstream filename;
    // std::setprecision을 사용하여 소수점 이하 2자리로 설정
    filename << metrics_dir << "/stanley_" << std::fixed << std::setprecision(3) << stanley_gain_ 
             << "_lookahead_" << lookahead_heading_ << "_Catalunya_raceline_metrics_" 
             << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    metrics_file_.open(filename.str());
    if (metrics_file_.is_open()) {
        // CSV 헤더 작성 (max 값 추가)
        metrics_file_ << "timestamp,cross_track_error,yaw_error,speed_error,"
                      << "max_cross_track_error,max_yaw_error,max_speed_error,"
                      << "current_x,current_y,current_yaw,current_speed,target_speed,closest_waypoint_idx\n";
        RCLCPP_INFO(this->get_logger(), "Metrics CSV file created: %s", filename.str().c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create metrics CSV file");
    }
}

float Auto_tuning::calculate_cross_track_error(float car_x, float car_y, size_t closest_idx) {
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

float Auto_tuning::calculate_yaw_error(float car_yaw, size_t closest_idx) {
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

void Auto_tuning::record_metrics(float car_x, float car_y, float car_yaw, float car_speed, size_t closest_idx, float target_speed) {
    // 현재 시간 계산 (시작 시간으로부터의 경과 시간)
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time_);
    double elapsed_seconds = elapsed.count() / 1000.0; // 초 단위
    
    EvaluationMetrics metrics;
    metrics.timestamp = elapsed_seconds; // 초 단위

    // 에러 계산
    metrics.cross_track_error = calculate_cross_track_error(car_x, car_y, closest_idx);
    metrics.yaw_error = calculate_yaw_error(car_yaw, closest_idx);
    metrics.speed_error = std::abs(car_speed - target_speed);

    // Max 값 업데이트
    max_cross_track_error_ = std::max(max_cross_track_error_, std::abs(metrics.cross_track_error));
    max_yaw_error_ = std::max(max_yaw_error_, std::abs(metrics.yaw_error));
    max_speed_error_ = std::max(max_speed_error_, metrics.speed_error);

    // 현재 상태 저장
    metrics.current_x = car_x;
    metrics.current_y = car_y;
    metrics.current_yaw = car_yaw;
    metrics.current_speed = car_speed;
    metrics.target_speed = target_speed;
    metrics.closest_waypoint_idx = closest_idx;

    // 메모리에 저장
    metrics_data_.push_back(metrics);

    // 실시간으로 CSV에도 저장 (max 값 포함)
    if (metrics_file_.is_open()) {
        metrics_file_ << metrics.timestamp << ","
                      << metrics.cross_track_error << ","
                      << metrics.yaw_error << ","
                      << metrics.speed_error << ","
                      << max_cross_track_error_ << ","
                      << max_yaw_error_ << ","
                      << max_speed_error_ << ","
                      << metrics.current_x << ","
                      << metrics.current_y << ","
                      << metrics.current_yaw << ","
                      << metrics.current_speed << ","
                      << metrics.target_speed << ","
                      << metrics.closest_waypoint_idx << "\n";
        metrics_file_.flush(); // 즉시 파일에 쓰기
    }

    // 로그 출력 (max 값 포함)
    RCLCPP_DEBUG(this->get_logger(), "Metrics - CTE: %.3f (Max: %.3f), YE: %.3f (Max: %.3f), SE: %.3f (Max: %.3f)", 
                 metrics.cross_track_error, max_cross_track_error_,
                 metrics.yaw_error, max_yaw_error_,
                 metrics.speed_error, max_speed_error_);
}

void Auto_tuning::create_new_csv_file() {
    // 기존 파일이 열려있으면 닫기
    if (metrics_file_.is_open()) {
        metrics_file_.close();
    }
    
    // 메트릭 데이터 초기화
    metrics_data_.clear();
    max_cross_track_error_ = 0.0f;
    max_yaw_error_ = 0.0f;
    max_speed_error_ = 0.0f;
    
    // 시작 시간 재설정
    start_time_ = std::chrono::steady_clock::now();
    
    // evaluation_metrics 폴더 생성
    std::string metrics_dir = "/home/jys/ROS2/f1thebeast_ws/src/auto_tuning/evaluation_metrics/csv_file/Catalunya";
    std::filesystem::create_directories(metrics_dir);

    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    // std::setprecision을 사용하여 소수점 이하 2자리로 설정
    filename << metrics_dir << "/stanley_" << std::fixed << std::setprecision(3) << stanley_gain_ 
             << "_lookahead_" << lookahead_heading_ << "_Catalunya_raceline_metrics_" 
             << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    // 현재 CSV 파일명 저장
    current_csv_filename_ = filename.str();

    metrics_file_.open(current_csv_filename_);
    if (metrics_file_.is_open()) {
        // CSV 헤더 작성
        metrics_file_ << "timestamp,cross_track_error,yaw_error,speed_error,"
                      << "max_cross_track_error,max_yaw_error,max_speed_error,"
                      << "current_x,current_y,current_yaw,current_speed,target_speed,closest_waypoint_idx\n";
        RCLCPP_INFO(this->get_logger(), "New CSV file created: %s", current_csv_filename_.c_str());
        RCLCPP_INFO(this->get_logger(), "Parameters - stanley_gain: %.1f, lookahead_heading: %d", 
                    stanley_gain_, lookahead_heading_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create new CSV file");
    }
}

void Auto_tuning::finalize_current_csv() {
    if (metrics_file_.is_open()) {
        // 최종 통계를 CSV 파일에 주석으로 추가
        if (!metrics_data_.empty()) {
            float avg_cte = 0.0f, avg_ye = 0.0f, avg_se = 0.0f;
            float rms_cte = 0.0f, rms_ye = 0.0f, rms_se = 0.0f;

            for (const auto& metrics : metrics_data_) {
                avg_cte += std::abs(metrics.cross_track_error);
                avg_ye += std::abs(metrics.yaw_error);
                avg_se += metrics.speed_error;

                rms_cte += metrics.cross_track_error * metrics.cross_track_error;
                rms_ye += metrics.yaw_error * metrics.yaw_error;
                rms_se += metrics.speed_error * metrics.speed_error;
            }

            size_t n = metrics_data_.size();
            avg_cte /= n; avg_ye /= n; avg_se /= n;
            rms_cte = std::sqrt(rms_cte / n);
            rms_ye = std::sqrt(rms_ye / n);
            rms_se = std::sqrt(rms_se / n);

            // CSV 파일에 통계 정보 추가 (주석으로)
            metrics_file_ << "\n# Final Statistics:\n";
            metrics_file_ << "# Stanley Gain: " << stanley_gain_ << "\n";
            metrics_file_ << "# Lookahead Heading: " << lookahead_heading_ << "\n";
            metrics_file_ << "# Cross Track Error - Avg: " << avg_cte << ", Max: " << max_cross_track_error_ << ", RMS: " << rms_cte << "\n";
            metrics_file_ << "# Yaw Error - Avg: " << avg_ye << ", Max: " << max_yaw_error_ << ", RMS: " << rms_ye << "\n";
            metrics_file_ << "# Speed Error - Avg: " << avg_se << ", Max: " << max_speed_error_ << ", RMS: " << rms_se << "\n";
            metrics_file_ << "# Total data points: " << n << "\n";
            
            // 로그 출력
            RCLCPP_INFO(this->get_logger(), "=== CSV File Finalized (LAP COMPLETED) ===");
            RCLCPP_INFO(this->get_logger(), "Parameters - stanley_gain: %.1f, lookahead_heading: %d", stanley_gain_, lookahead_heading_);
            RCLCPP_INFO(this->get_logger(), "Cross Track Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_cte, max_cross_track_error_, rms_cte);
            RCLCPP_INFO(this->get_logger(), "Total data points: %zu", n);
            RCLCPP_INFO(this->get_logger(), "CSV file saved: %s", current_csv_filename_.c_str());
        }
        
        metrics_file_.close();
    }
}

void Auto_tuning::delete_current_csv() {
    // 파일이 열려있으면 닫기
    if (metrics_file_.is_open()) {
        metrics_file_.close();
    }
    
    // 파일 삭제
    if (!current_csv_filename_.empty()) {
        try {
            if (std::filesystem::exists(current_csv_filename_)) {
                std::filesystem::remove(current_csv_filename_);
                RCLCPP_WARN(this->get_logger(), "=== CSV File DELETED (COLLISION) ===");
                RCLCPP_WARN(this->get_logger(), "Deleted file: %s", current_csv_filename_.c_str());
                RCLCPP_WARN(this->get_logger(), "Parameters that failed - stanley_gain: %.1f, lookahead_heading: %d", 
                           stanley_gain_, lookahead_heading_);
            }
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to delete CSV file: %s", e.what());
        }
    }
    
    // 메트릭 데이터 초기화
    metrics_data_.clear();
    max_cross_track_error_ = 0.0f;
    max_yaw_error_ = 0.0f;
    max_speed_error_ = 0.0f;
    current_csv_filename_.clear();
}

// 최종 버전의 save_metrics_to_csv (소멸자용)
void Auto_tuning::save_metrics_to_csv() {
    // 소멸자에서 호출되는 경우를 위해 finalize_current_csv 호출 (완주한 경우만)
    if (!metrics_data_.empty()) {
        finalize_current_csv();
    }
}