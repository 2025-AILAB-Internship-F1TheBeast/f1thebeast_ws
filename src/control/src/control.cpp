#include "control.hpp"
#include <filesystem>

Control::Control(float stanley_gain) : Node("controller_node"), odom_topic("/odom"), drive_topic("/drive"), 
    pid_integral_(0.0f), pid_prev_error_(0.0f), stanley_gain_(stanley_gain) {
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    lookahead_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_waypoints_marker", 1);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&Control::odom_callback, this, std::placeholders::_1));

    // raceline.csv로 경로 변경
    std::string raceline_csv_path = "/home/jys/ROS2/f1thebeast_ws/src/control/map/f1tenth_racetracks/Spielberg/Spielberg_raceline.csv";
    load_raceline_waypoints(raceline_csv_path);

    // Evaluation metrics 초기화
    initialize_metrics_csv();
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Control node initialized with stanley gain: %.2f", stanley_gain_);
}

Control::~Control() {
    // CSV 파일 저장 및 닫기
    save_metrics_to_csv();
    if (metrics_file_.is_open()) {
        metrics_file_.close();
    }
}

float Control::pure_pursuit(float steer_ang_rad, float lookahead_dist) {
    const float lidar_to_rear = 0.27f;
    const float wheel_base = 0.32f;

    float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
    float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);

    float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
    float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + std::pow(bestpoint_y, 2));

    return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
}

float Control::stanley(float car_velocity, std::vector<std::pair<float, float>>& waypoints) {
    const float k = stanley_gain_; // 멤버 변수 사용
    const float x_front = 0.0f;
    const float y_front = 0.0f;
    
    // waypoints 정보
    float waypoint_x1 = 0.0f, waypoint_y1 = 0.0f;
    float waypoint_x2 = 0.0f, waypoint_y2 = 0.0f;

    // 첫 번째 waypoint는 waypoints의 x값이 처음으로 양수가 되는 지점
    // 두 번째 waypoint는 그 다음 waypoint
    for (size_t i = 0; i < waypoints.size(); ++i) {
        // 첫 번째 waypoint의 x값이 양수인 경우
        if (waypoints[i].first > 0.0f) {
            waypoint_x1 = waypoints[i].first;
            waypoint_y1 = waypoints[i].second;
            waypoint_x2 = waypoints[i + 1].first;
            waypoint_y2 = waypoints[i + 1].second;
            std::cout << "Using waypoints: (" << waypoint_x1 << ", " << waypoint_y1 << "), ("
                      << waypoint_x2 << ", " << waypoint_y2 << ")" << std::endl;
            break;
        }
    }

    float track_error = point_to_line_distance(waypoint_x1, waypoint_y1, waypoint_x2, waypoint_y2, x_front, y_front);

    if ((waypoint_y1 + waypoint_y2) / 2.0f < 0.0f) {
        track_error *= -1.0;
    }
    
    std::cout << "Track error: " << track_error << ", Stanley gain: " << k << std::endl;
    float heading_error = std::atan2(waypoint_y2 - waypoint_y1, waypoint_x2 - waypoint_x1) - std::atan2(y_front, x_front);
    std::cout << "Heading error: " << heading_error << std::endl;
    float steering_angle = heading_error + std::atan2(k * track_error, car_velocity);

    std::cout << "Steering Angle: " << steering_angle*180.0/PI << std::endl;
    return steering_angle;
}

float Control::point_to_line_distance(float x1, float y1, float x2, float y2, float px, float py) {
    float a = y2 - y1;
    float b = x1 - x2;
    float c = x2 * y1 - x1 * y2;
    float distance = std::abs(a * px + b * py + c) / std::sqrt(a * a + b * b);
    return distance;
}

std::pair<float, float> Control::vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints, size_t closest_idx) {
    
    // local_path의 좌표를 global 좌표계에서 local 좌표계로 변환
    std::vector<std::pair<float, float>> local_points = global_to_local(global_car_x, global_car_y, yaw, waypoints);
    float current_speed = car_speed;

    // local_path의 좌표를 모두 출력
    for (const auto& point : local_points) {
        std::cout << "Local Point: (" << point.first << ", " << point.second << ")" << std::endl;
    }

    // raceline의 속도 값을 목표 속도로 사용
    float target_speed = waypoints[closest_idx].vx;
    float drive_speed = pid_controller(target_speed, current_speed);
    float stanley_steer = stanley(current_speed, local_points);

    return std::make_pair(stanley_steer, drive_speed);
}

float Control::pid_controller(float target_speed, float current_speed) {
    float dt = 0.1;

    const float Kp = 2.0f;
    const float Ki = 1.1f;
    const float Kd = 1.2f;

    float error = target_speed - current_speed;

    float P = Kp * error;
    pid_integral_ += error * dt;
    float I = Ki * pid_integral_;
    
    float derivative = (error - pid_prev_error_) / dt;
    float D = Kd * derivative;

    float output = P + I + D;
    pid_prev_error_ = error;

    std::cout << "Target Speed: " << target_speed << ", Current Speed: " << current_speed << ", PID Output: " << output << std::endl;

    return output;
}

void Control::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    try {
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
        
        RCLCPP_INFO(this->get_logger(), "Current Pose (front_wheel): (%.2f, %.2f, %.2f), Speed: %.2f",
                    global_current_x, global_current_y, global_current_yaw, car_current_speed);
        
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

        std::vector<RacelineWaypoint> lookahead_waypoints;
        for (size_t i = 0; i < 10; ++i) {
            size_t idx = closest_idx + i;
            if (idx < global_raceline_waypoints_.size()) {
                lookahead_waypoints.push_back(global_raceline_waypoints_[idx]);
            }
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
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during odom_callback: %s", e.what());
    }
}

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

void Control::publish_lookahead_waypoints_marker(const std::vector<RacelineWaypoint>& lookahead_waypoints) {
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

std::vector<std::pair<float, float>> Control::global_to_local(float car_x, float car_y, float car_yaw, const std::vector<RacelineWaypoint>& waypoints) {
    std::vector<std::pair<float, float>> local_points;
    float cos_yaw = std::cos(-car_yaw);
    float sin_yaw = std::sin(-car_yaw);

    for (const auto& wp : waypoints) {
        float dx = wp.x - car_x;
        float dy = wp.y - car_y;
        float local_x = dx * cos_yaw - dy * sin_yaw;
        float local_y = dx * sin_yaw + dy * cos_yaw;
        local_points.emplace_back(local_x, local_y);
    }
    return local_points;
}

void Control::initialize_metrics_csv() {
    // evaluation_metrics 폴더 생성
    std::string metrics_dir = "/home/jys/ROS2/f1thebeast_ws/src/control/evaluation_metrics";
    std::filesystem::create_directories(metrics_dir);

    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    filename << metrics_dir << "/metrics_" 
             << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";

    metrics_file_.open(filename.str());
    if (metrics_file_.is_open()) {
        // CSV 헤더 작성
        metrics_file_ << "timestamp,cross_track_error,yaw_error,speed_error,"
                      << "current_x,current_y,current_yaw,current_speed,target_speed,closest_waypoint_idx\n";
        RCLCPP_INFO(this->get_logger(), "Metrics CSV file created: %s", filename.str().c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create metrics CSV file");
    }
}

float Control::calculate_cross_track_error(float car_x, float car_y, size_t closest_idx) {
    if (closest_idx >= global_raceline_waypoints_.size() - 1) {
        return 0.0f;
    }

    // 현재 waypoint와 다음 waypoint를 이용해 라인 생성
    float x1 = global_raceline_waypoints_[closest_idx].x;
    float y1 = global_raceline_waypoints_[closest_idx].y;
    float x2 = global_raceline_waypoints_[closest_idx + 1].x;
    float y2 = global_raceline_waypoints_[closest_idx + 1].y;

    // 점과 직선 사이의 거리 계산
    return point_to_line_distance(x1, y1, x2, y2, car_x, car_y);
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

    yaw_error *= 180.0 / PI; // 라디안에서 도 단위로 변환

    return yaw_error;
}

void Control::record_metrics(float car_x, float car_y, float car_yaw, float car_speed, size_t closest_idx, float target_speed) {
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

    // 현재 상태 저장
    metrics.current_x = car_x;
    metrics.current_y = car_y;
    metrics.current_yaw = car_yaw;
    metrics.current_speed = car_speed;
    metrics.target_speed = target_speed;
    metrics.closest_waypoint_idx = closest_idx;

    // 메모리에 저장
    metrics_data_.push_back(metrics);

    // 실시간으로 CSV에도 저장
    if (metrics_file_.is_open()) {
        metrics_file_ << metrics.timestamp << ","
                      << metrics.cross_track_error << ","
                      << metrics.yaw_error << ","
                      << metrics.speed_error << ","
                      << metrics.current_x << ","
                      << metrics.current_y << ","
                      << metrics.current_yaw << ","
                      << metrics.current_speed << ","
                      << metrics.target_speed << ","
                      << metrics.closest_waypoint_idx << "\n";
        metrics_file_.flush(); // 즉시 파일에 쓰기
    }

    // 로그 출력 (선택사항)
    RCLCPP_DEBUG(this->get_logger(), "Metrics - CTE: %.3f, YE: %.3f, SE: %.3f", 
                 metrics.cross_track_error, metrics.yaw_error, metrics.speed_error);
}

void Control::save_metrics_to_csv() {
    if (!metrics_data_.empty()) {
        // 통계 계산
        float avg_cte = 0.0f, avg_ye = 0.0f, avg_se = 0.0f;
        float max_cte = 0.0f, max_ye = 0.0f, max_se = 0.0f;
        float rms_cte = 0.0f, rms_ye = 0.0f, rms_se = 0.0f;

        for (const auto& metrics : metrics_data_) {
            avg_cte += std::abs(metrics.cross_track_error);
            avg_ye += std::abs(metrics.yaw_error);
            avg_se += metrics.speed_error;

            max_cte = std::max(max_cte, std::abs(metrics.cross_track_error));
            max_ye = std::max(max_ye, std::abs(metrics.yaw_error));
            max_se = std::max(max_se, metrics.speed_error);

            rms_cte += metrics.cross_track_error * metrics.cross_track_error;
            rms_ye += metrics.yaw_error * metrics.yaw_error;
            rms_se += metrics.speed_error * metrics.speed_error;
        }

        size_t n = metrics_data_.size();
        avg_cte /= n; avg_ye /= n; avg_se /= n;
        rms_cte = std::sqrt(rms_cte / n);
        rms_ye = std::sqrt(rms_ye / n);
        rms_se = std::sqrt(rms_se / n);

        // 통계 출력
        RCLCPP_INFO(this->get_logger(), "=== Evaluation Metrics Summary ===");
        RCLCPP_INFO(this->get_logger(), "Cross Track Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_cte, max_cte, rms_cte);
        RCLCPP_INFO(this->get_logger(), "Yaw Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_ye, max_ye, rms_ye);
        RCLCPP_INFO(this->get_logger(), "Speed Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_se, max_se, rms_se);
        RCLCPP_INFO(this->get_logger(), "Total data points: %zu", n);
    }
}