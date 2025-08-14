/*
csv 파일 기록 관련 함수들
*/
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