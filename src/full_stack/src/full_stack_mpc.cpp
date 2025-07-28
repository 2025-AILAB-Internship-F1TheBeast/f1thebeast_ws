#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // ← .hpp로!
#include <tf2/utils.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Waypoint {
    float x;
    float y;
    float yaw;
    float v;
};

class FullStackMPC : public rclcpp::Node {
public:
    FullStackMPC() : Node("full_stack_mpc") {
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&FullStackMPC::odom_callback, this, std::placeholders::_1));
        local_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("local_waypoints", 1);
        load_waypoints("/home/jys/ROS2/f1thebeast_ws/src/full_stack/map/Spielberg_centerline.csv");
        RCLCPP_INFO(this->get_logger(), "MPC node initialized");
    }

private:
    // Vehicle parameters
    const float wheelbase_ = 0.33f;
    const float dt_ = 0.1f;
    const int horizon_ = 8; // TK
    const int nx_ = 4; // [x, y, v, yaw]
    const int nu_ = 2; // [accel, steer]
    // Cost matrices (Q, R)
    MatrixXd Q_ = (MatrixXd(4,4) << 13.5,0,0,0, 0,13.5,0,0, 0,0,5.5,0, 0,0,0,13.0).finished();
    MatrixXd R_ = (MatrixXd(2,2) << 0.01,0, 0,100.0).finished();

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_waypoint_pub_;
    std::vector<Waypoint> waypoints_;

    void load_waypoints(const std::string& path) {
        std::ifstream file(path);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            float x = std::stof(x_str);
            float y = std::stof(y_str);
            waypoints_.push_back({x, y, 0.0f, 4.0f}); // yaw, v는 임시값
        }
        // yaw, v 계산 (간단히)
        for (size_t i = 1; i < waypoints_.size(); ++i) {
            float dx = waypoints_[i].x - waypoints_[i-1].x;
            float dy = waypoints_[i].y - waypoints_[i-1].y;
            waypoints_[i-1].yaw = std::atan2(dy, dx);
            waypoints_[i-1].v = 4.0f; // 원하는 속도 (임의)
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        float px = msg->pose.pose.position.x;
        float py = msg->pose.pose.position.y;
        float vx = msg->twist.twist.linear.x;
        float vy = msg->twist.twist.linear.y;
        float v = std::sqrt(vx*vx + vy*vy);
        float yaw = tf2::getYaw(msg->pose.pose.orientation);

        // 가장 가까운 waypoint 인덱스 찾기
        size_t closest_idx = 0;
        float min_dist = 1e9;
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            float dx = px - waypoints_[i].x;
            float dy = py - waypoints_[i].y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // MPC용 참조 trajectory 추출
        std::vector<Waypoint> ref_traj;
        for (int i = 0; i < horizon_+1; ++i) {
            size_t idx = std::min(closest_idx + i, waypoints_.size()-1);
            ref_traj.push_back(waypoints_[idx]);
        }

        // 현재 상태 벡터
        VectorXd state(nx_);
        state << px, py, v, yaw;

        // 참조 trajectory 행렬
        MatrixXd ref(nx_, horizon_+1);
        for (int i = 0; i < horizon_+1; ++i) {
            ref(0,i) = ref_traj[i].x;
            ref(1,i) = ref_traj[i].y;
            ref(2,i) = ref_traj[i].v;
            ref(3,i) = ref_traj[i].yaw;
        }

        // MPC 최적화 (여기서는 단순하게 첫 step만 계산, 실제론 QP solver 필요)
        float steer = 0.0f, accel = 0.0f;
        mpc_solve(state, ref, steer, accel);

        // 메시지 publish
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = steer;
        drive_msg.drive.speed = v + accel * dt_;
        drive_pub_->publish(drive_msg);

        // 로컬 웨이포인트 시각화를 위한 마커 메시지 publish
        // local waypoint marker publish
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "local_waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto& wp : ref_traj) {
            geometry_msgs::msg::Point p;
            p.x = wp.x;
            p.y = wp.y;
            p.z = 0.1;
            marker.points.push_back(p);
        }
        local_waypoint_pub_->publish(marker);
    }

    // 간단한 선형화된 모델 기반 예시 (실제론 QP solver 필요)
    void mpc_solve(const VectorXd& state, const MatrixXd& ref, float& steer, float& accel) {
        // 선형화된 모델: x[k+1] = x[k] + v*dt*cos(yaw), y[k+1] = y[k] + v*dt*sin(yaw), ...
        // 여기서는 horizon=1만 사용, 실제론 QP로 horizon 전체 최적화 필요
        VectorXd u(nu_);
        u.setZero();
        // 단순히 현재와 참조의 차이로 P제어 (실제론 QP)
        VectorXd error = ref.col(1) - state;
        accel = 1.0f * error(2); // 속도 오차
        steer = 0.8f * (error(3)); // yaw 오차
        // Saturation
        if (accel > 5.0f) accel = 5.0f;
        if (accel < -5.0f) accel = -5.0f;
        if (steer > 0.4189f) steer = 0.4189f;
        if (steer < -0.4189f) steer = -0.4189f;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullStackMPC>());
    rclcpp::shutdown();
    return 0;
}