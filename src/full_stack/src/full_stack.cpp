/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


#ifndef PI
#define PI 3.14159265358979323846
#endif

// Waypoint 구조체 정의
struct Waypoint {
    float x;
    float y;
};

/// CHECK: include needed ROS msg type headers and libraries
class F1FullStack : public rclcpp::Node
{
    // Implement Reactive Follow Gap on the car
    // This is just a template, you are free to implement your own node!
  public:
    rclcpp::TimerBase::SharedPtr marker_timer_;

    F1FullStack() : Node("f1_node")
    {
        /// TODO: create ROS subscribers and publishers
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        lookahead_waypoints_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lookahead_waypoints_marker", 1);

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&F1FullStack::odom_callback, this, std::placeholders::_1));

        std::string centerline_csv_path = "/home/jys/ROS2/f1thebeast_ws/src/full_stack/map/Spielberg_centerline.csv";
        load_centerline_waypoints(centerline_csv_path);

        RCLCPP_INFO(this->get_logger(), "F1FullStack node initialized");
    }

  private:
    std::string odom_topic = "/odom";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_waypoints_marker_pub_;

    // PID 관련 멤버 변수
    float pid_integral_ = 0.0f;
    float pid_prev_error_ = 0.0f;

    // 전역 변수: 로드된 웨이포인트
    std::vector<Waypoint> global_waypoints_;

    // 횡 제어 : Pure Pursuit 알고리즘
    float pure_pursuit(float steer_ang_rad, float lookahead_dist)
    {
        const float lidar_to_rear = 0.27f;
        const float wheel_base = 0.32f;

        float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
        float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);

        float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
        float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + std::pow(bestpoint_y, 2));

        // Final Pure Pursuit Angle
        return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
    }

    // 횡 제어 : Stanley 알고리즘
    float stanley(float car_velocity, std::vector<std::pair<float, float>>& waypoints)
    {
        const float k = 2.0f; // Stanley controller gain
        // const float wheel_base = 0.32f;
        const float x_front = 0.05f;
        const float y_front = 0.0f;

        float waypoint_x1 = waypoints[0].first;
        float waypoint_y1 = waypoints[0].second;
        float waypoint_x2 = waypoints[1].first;
        float waypoint_y2 = waypoints[1].second;
        
        std::cout << "Waypoints: (" << waypoint_x1 << ", " << waypoint_y1 << "), ("
                  << waypoint_x2 << ", " << waypoint_y2 << ")" << std::endl;

        float track_error = point_to_line_distance(waypoint_x1, waypoint_y1, waypoint_x2, waypoint_y2, x_front, y_front);

        // track error를 계산할 때, 단순히 점과 직선 사이의 거리를 구하면 안됨.
        // Why? 왜냐하면 local 좌표계 기준으로 waypoint가 y< 0이면 track error가 음수로 나와야하는데, 거리만 구하면 무조건 양수로 나오기 때문.
        // 따라서 두 waypoint의 y의 평균이 0보다 작은지 확인하고, track error의 부호를 결정해야함.
        if ((waypoint_y1 + waypoint_y2) / 2.0f < 0.0f) {
            track_error *= -1.0; // y < 0
        }
        
        std::cout  << "Track error: " << track_error << std::endl;
        float heading_error = std::atan2(waypoint_y2 - waypoint_y1, waypoint_x2 - waypoint_x1) - std::atan2(y_front, x_front);
        std::cout << "Heading error: " << heading_error << std::endl;
        float steering_angle = heading_error + std::atan2(k * track_error, car_velocity);

        std::cout << "Steering Angle: " << steering_angle*180.0/PI << std::endl;
        return steering_angle;
    }

    // 점과 직선 사이의 거리 구하는 공식
    float point_to_line_distance(float x1, float y1, float x2, float y2, float px, float py)
    {
        float numerator = std::abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1);
        float denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
        return numerator / denominator;
    }

    // 차량 제어 파트
    std::pair<float, float> vehicle_control(float global_car_x, float global_car_y, float yaw, float car_speed, const std::vector<Waypoint>& waypoints)
    {
        // 좌표 변환 시행
        std::vector<std::pair<float, float>> local_points = global_to_local(global_car_x, global_car_y, yaw, waypoints);
        float current_speed = car_speed; // 현재 속도

        // PID 컨트롤러를 사용하여 control speed 설정
        float drive_speed = pid_controller(local_points, current_speed);

        // stanley_steer 변수는 Stanley method에서 delta 즉, 차량의 스티어링 회전 값을 의미.
        float stanley_steer = stanley(current_speed, local_points);

        return std::make_pair(stanley_steer, drive_speed);
    }

    // PID 컨트롤러 함수: 목표 속도와 현재 속도를 받아 linear speed를 반환
    float pid_controller(std::vector<std::pair<float, float>>& waypoints, float current_speed)
    {
        // 목표 속도를 계산
        // waypoints의 첫 번째 점과 두 번째 점을 사용하여 목표 속도를 계산
        float dt = 0.1;
        float dx = waypoints[1].first - waypoints[0].first;
        float dy = waypoints[1].second - waypoints[0].second;
        float target_speed = std::sqrt(dx * dx + dy * dy) / dt; // 0.1초 동안의 거리로 속도 계산

        // PID 파라미터 (상황에 맞게 튜닝)
        const float Kp = 2.0f;
        const float Ki = 1.1f;
        const float Kd = 1.2f;

        float error = target_speed - current_speed;

        // P값 계산
        float P = Kp * error;
        

        // I값 계산
        pid_integral_ += error * dt;
        float I = Ki * pid_integral_;
        

        // D값 계산
        float derivative = (error - pid_prev_error_) / dt;
        float D = Kd * derivative;

        // PID 출력 계산
        float output = P + I + D;
        pid_prev_error_ = error;

        return output;
    }

    // 인지 판단 통합 파트
    // 현재 완전히 구성된 slam 패키지나 planner 패키지가 없으므로 nav_msgs::msg::Odometry를 받아 localization 및 local_path를 제어에 넘기도록 코드를 구성
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        try
        {
            // 차량의 현재 pose (글로벌 좌표계)
            float global_current_x = odom_msg->pose.pose.position.x;
            float global_current_y = odom_msg->pose.pose.position.y;
            float global_current_yaw = tf2::getYaw(odom_msg->pose.pose.orientation);
            float car_current_speed = std::sqrt(
                odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
                odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y);
            RCLCPP_INFO(this->get_logger(), "Current Pose: (%.2f, %.2f, %.2f), Speed: %.2f",
                        global_current_x, global_current_y, global_current_yaw, car_current_speed);
            
            // 가장 가까운 waypoint 찾기
            size_t closest_idx = 0;
            float min_dist = std::numeric_limits<float>::max();
            for (size_t i = 0; i < global_waypoints_.size(); ++i) {
                float dx = global_waypoints_[i].x - global_current_x;
                float dy = global_waypoints_[i].y - global_current_y;
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_idx = i;
                }
            }

            // 뒤에 9개 점까지 추출 (총 10개)
            std::vector<Waypoint> lookahead_waypoints;
            for (size_t i = 0; i < 10; ++i) {
                size_t idx = closest_idx + i;
                if (idx < global_waypoints_.size()) {
                    lookahead_waypoints.push_back(global_waypoints_[idx]);
                }
            }

            // lookahead waypoint marker publish
            publish_lookahead_waypoints_marker(lookahead_waypoints);

            // 차량 제어 함수 호출
            auto [steering_angle, drive_speed] = vehicle_control(global_current_x, global_current_y, global_current_yaw, car_current_speed, lookahead_waypoints);

            // drive 메시지 publish
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = drive_speed;
            drive_publisher_->publish(drive_msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during odom_callback: %s", e.what());
        }
    }

    // CSV에서 waypoint 읽어오는 함수
    void load_centerline_waypoints(const std::string& csv_path) {
        std::ifstream file(csv_path);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            global_waypoints_.push_back({std::stof(x_str), std::stof(y_str)});
        }
    }

    // 10개 lookahead waypoint를 삼각형(빨간색) marker로 표시하는 함수
    void publish_lookahead_waypoints_marker(const std::vector<Waypoint>& lookahead_waypoints)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "lookahead_waypoints";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // 각 waypoint마다 삼각형 하나씩 생성 (작은 삼각형)
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

    std::vector<std::pair<float, float>> global_to_local(float car_x, float car_y, float car_yaw, const std::vector<Waypoint>& waypoints) {
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<F1FullStack>());
    rclcpp::shutdown();
    return 0;
}