/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
#include "control_real.hpp"

int main(int argc, char **argv) {
    // ROS2 초기화
    rclcpp::init(argc, argv);
    
    // 기본 ini 파일 경로
    std::string ini_file_path = "/home/jys/ROS2/f1thebeast_ws/src/control_ws/control_real/config/param.ini";
    
    // Control 노드 생성
    auto node = std::make_shared<Control>(ini_file_path);
    
    // SingleThreadedExecutor 사용 (스레드 안전성을 위해)
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}