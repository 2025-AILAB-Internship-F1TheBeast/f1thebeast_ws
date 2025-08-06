#include "control.hpp"
#include <filesystem>

Control::Control(float stanley_gain, bool enable_metrics) : Node("controller_node"), odom_topic("/odom"), drive_topic("/drive"), 
    pid_integral_(0.0f), pid_prev_error_(0.0f), stanley_gain_(stanley_gain), current_closest_idx_(0), previous_velocity_(1.0f),
    previous_time_ns(this->now()), enable_metrics_(enable_metrics), max_cross_track_error_(0.0f), max_yaw_error_(0.0f), max_speed_error_(0.0f) {
    
    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
    lookahead_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_waypoints_marker", 1);

    // raceline.csv로 경로 변경
    std::string raceline_csv_path = "/home/jys/ROS2/f1thebeast_ws/src/control/map/f1tenth_racetracks/Catalunya/Catalunya_raceline.csv";
    load_raceline_waypoints(raceline_csv_path);

    // Evaluation metrics 초기화 (enable_metrics가 true일 때만)
    if (enable_metrics_) {
        initialize_metrics_csv();
        start_time_ = std::chrono::steady_clock::now();
    }

    // 초기 위치 설정을 위한 일회성 odometry 구독
    initial_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&Control::initial_odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Control node initialized with stanley gain: %.2f, metrics: %s", stanley_gain_, enable_metrics_ ? "enabled" : "disabled");
}

Control::~Control() {
    // CSV 파일 저장 및 닫기 (metrics가 활성화된 경우에만)
    if (enable_metrics_) {
        save_metrics_to_csv();
        if (metrics_file_.is_open()) {
            metrics_file_.close();
        }
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

float Control::local_planner_based_stanley_controller(float car_velocity, std::vector<LocalWaypoint>& waypoints) {
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

    float waypoint_x = waypoints[0].x;
    float waypoint_y = waypoints[0].y;
    float waypoint_heading = waypoints[0].heading;

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

float Control::stanley_controller(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints) {

    // Hyperparameter for stanley controller
    const float k_dist_gain = stanley_gain_;

    // waypoint들 간의 벡터를 저장 (diffs = trajectory[1:,:] - trajectory[:-1,:])
    std::vector<DifferentialWaypoint> diffs;

    // waypoints들 사이의 벡터를 계산
    for (int i = 0; i < waypoints.size() - 1; i++) {
        float dx = waypoints[i+1].x - waypoints[i].x;
        float dy = waypoints[i+1].y - waypoints[i].y;
        float l2s = dx * dx + dy * dy;  // 벡터 길이의 제곱
        diffs.emplace_back(dx, dy, l2s);
    }

    // 내적값을 저장 (dots[i] = np.dot((point - trajectory[i, :]), diffs[i, :]))
    std::vector<float> dots;
    for (int i = 0; i < diffs.size(); i++) {
        float to_point_x = global_car_x - waypoints[i].x;
        float to_point_y = global_car_y - waypoints[i].y;
        float dot_product = to_point_x * diffs[i].dx + to_point_y * diffs[i].dy;
        dots.push_back(dot_product);
    }

    // t 값 계산 (t = dots / l2s)
    std::vector<float> t;
    for (int i = 0; i < dots.size(); i++) {
        float t_val = dots[i] / diffs[i].l2s;
        // t 값을 [0, 1] 범위로 클램핑
        if (t_val < 0.0f) t_val = 0.0f;
        if (t_val > 1.0f) t_val = 1.0f;
        t.push_back(t_val);
    }

    // 각 선분에서의 projection point 계산 (projections = trajectory[:-1,:] + (t*diffs.T).T)
    std::vector<std::pair<float, float>> projections;
    for (int i = 0; i < t.size(); i++) {
        float proj_x = waypoints[i].x + t[i] * diffs[i].dx;
        float proj_y = waypoints[i].y + t[i] * diffs[i].dy;
        projections.emplace_back(proj_x, proj_y);
    }

    // 각 projection point까지의 거리 계산
    std::vector<float> dists;
    for (int i = 0; i < projections.size(); i++) {
        float temp_x = global_car_x - projections[i].first;
        float temp_y = global_car_y - projections[i].second;
        float dist = std::sqrt(temp_x * temp_x + temp_y * temp_y);
        dists.push_back(dist);
    }

    // 가장 가까운 선분 찾기
    int min_dist_segment = 0;
    float min_dist = dists[0];
    for (int i = 1; i < dists.size(); i++) {
        if (dists[i] < min_dist) {
            min_dist = dists[i];
            min_dist_segment = i;
        }
    }

    // Cross track error 계산 (가장 가까운 점까지의 거리)
    float track_error = min_dist;
    
    // 부호 결정: Python 코드 참고 - 차량의 heading을 기준으로 판단
    // 차량 위치에서 가장 가까운 projection point로의 벡터
    float vec_dist_x = global_car_x - projections[min_dist_segment].first;
    float vec_dist_y = global_car_y - projections[min_dist_segment].second;
    
    // 차량 heading에서 90도 회전한 벡터 (차량의 왼쪽 방향)
    float front_axle_vec_rot_90_x = std::cos(yaw - PI/2.0f);
    float front_axle_vec_rot_90_y = std::sin(yaw - PI/2.0f);
    
    // 내적을 이용한 cross track error 부호 결정
    // ef = np.dot(vec_dist_nearest_point.T, front_axle_vec_rot_90)
    float ef_dot_product = vec_dist_x * front_axle_vec_rot_90_x + vec_dist_y * front_axle_vec_rot_90_y;
    
    // ef_dot_product가 양수면 차량이 경로의 왼쪽에 있음 (Stanley controller에서는 양수)
    if (ef_dot_product < 0) {
        track_error *= -1.0f;  // 차량이 경로의 오른쪽에 있으면 음수
    }

    // Heading error 계산 (가장 가까운 선분의 heading 사용)
    // float segment_heading = std::atan2(diffs[min_dist_segment].dy, diffs[min_dist_segment].dx);
    float segment_heading = waypoints[min_dist_segment].psi;

    // segment_heading -PI에서 PI 범위로 클램핑
    if (segment_heading >= PI) {
        segment_heading -= 2 * PI;
    } else if (segment_heading < -PI) {
        segment_heading += 2 * PI;
    }

    // std::cout << "segment_heading : " << segment_heading * 180 / PI << "도" << std::endl;
    // std::cout << "차량의 yaw : " << yaw * 180 / PI << "도" << std::endl;
    float heading_error = segment_heading - yaw;  // 차량의 yaw와 선분의 heading 차이

    // std::cout << "heading_error (before clamping): " << heading_error * 180 / PI << "도" << std::endl;

    // heading error를 -PI에서 PI 범위로 클램핑
    if (heading_error > PI) {
        heading_error -= 2 * PI;
    } else if (heading_error < -PI) {
        heading_error += 2 * PI;
    }

    // Stanley controller 공식: steering_angle = heading_error + atan2(k * cross_track_error, velocity)
    float steering_angle = heading_error + std::atan2(k_dist_gain * track_error, car_speed+0.1f);

    std::cout << "steering_angle : " << steering_angle * 180 / PI << "도" << std::endl;
    return steering_angle;
}

// 새로운 point_to_line_distance_with_heading 함수 (heading 사용)
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

std::pair<float, float> Control::vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<RacelineWaypoint>& waypoints, size_t closest_idx) {

    // raceline의 속도 값을 목표 속도로 사용
    float target_speed = waypoints[3].vx;
    float drive_speed = target_speed * 0.4f;  // 속도 조정 비율 (예: 1.0f로 설정)

    // stanley controller 호출해서 스티어링 각도 계산
    float stanley_steer = stanley_controller(global_car_x, global_car_y, yaw, car_speed, waypoints);
    return std::make_pair(stanley_steer, drive_speed);
}

float Control::pid_controller(float target_speed, float current_speed) {
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

void Control::initial_odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
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
            "/ego_racecar/odom", 10, std::bind(&Control::odom_callback, this, std::placeholders::_1));
        
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error during initial_odom_callback: %s", e.what());
    }
}

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
    if (current_closest_idx_ > total_waypoints - 50) {  // 끝 부분에 가까워지면
        RCLCPP_INFO(this->get_logger(), "Near end: current_idx=%zu, new_idx=%zu, total=%zu, dist=%.2f", 
                    current_closest_idx_, closest_idx, total_waypoints, min_dist);
    }
    
    return closest_idx;
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

        // 효율적인 가장 가까운 waypoint 찾기 (순환 경로 고려)
        size_t closest_idx = find_closest_waypoint_local_search(global_current_x, global_current_y);
        current_closest_idx_ = closest_idx; // 현재 인덱스 업데이트

        // lookahead waypoints 생성
        // 앞 뒤로 3개씩의 waypoints, 총 7개를 생성

        std::vector<RacelineWaypoint> lookahead_waypoints;
        for (int i = -3; i < 4; i++) {
            size_t idx = (closest_idx + i + global_raceline_waypoints_.size()) % global_raceline_waypoints_.size();  // 순환 인덱스
            lookahead_waypoints.push_back(global_raceline_waypoints_[idx]);
        }

        publish_lookahead_waypoints_marker(lookahead_waypoints);

        auto [steering_angle, drive_speed] = vehicle_control(global_current_x, global_current_y, global_current_yaw, car_current_speed, lookahead_waypoints, 0);

        float target_speed = global_raceline_waypoints_[closest_idx].vx;

        // Evaluation metrics 기록 (metrics가 활성화된 경우에만)
        if (enable_metrics_) {
            record_metrics(global_current_x, global_current_y, global_current_yaw, car_current_speed, closest_idx, target_speed);
        }

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
        
        // 헤딩 변환
        float local_heading = wp.psi - car_yaw;
        while (local_heading > M_PI) local_heading -= 2 * M_PI;
        while (local_heading < -M_PI) local_heading += 2 * M_PI;
        
        local_points.emplace_back(local_x, local_y, local_heading);
    }
    return local_points;
}

void Control::initialize_metrics_csv() {
    // evaluation_metrics 폴더 생성
    std::string metrics_dir = "/home/jys/ROS2/f1thebeast_ws/src/control/evaluation_metrics/csv_file";
    std::filesystem::create_directories(metrics_dir);

    // 현재 시간으로 파일명 생성
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    
    std::stringstream filename;
    filename << metrics_dir << "/" << stanley_gain_ << "_" << "_Catalunya_metrics_"
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

void Control::save_metrics_to_csv() {
    if (!metrics_data_.empty()) {
        // 통계 계산
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

        // 통계 출력 (최종 max 값 사용)
        RCLCPP_INFO(this->get_logger(), "=== Evaluation Metrics Summary ===");
        RCLCPP_INFO(this->get_logger(), "Cross Track Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_cte, max_cross_track_error_, rms_cte);
        RCLCPP_INFO(this->get_logger(), "Yaw Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_ye, max_yaw_error_, rms_ye);
        RCLCPP_INFO(this->get_logger(), "Speed Error - Avg: %.3f, Max: %.3f, RMS: %.3f", avg_se, max_speed_error_, rms_se);
        RCLCPP_INFO(this->get_logger(), "Total data points: %zu", n);
    }
}