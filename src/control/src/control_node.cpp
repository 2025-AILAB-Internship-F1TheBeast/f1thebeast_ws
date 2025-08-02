/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
#include "control.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    float stanley_gain = 13.0f; // 기본값
    
    // Command line argument 파싱
    if (argc > 1) {
        try {
            stanley_gain = std::stof(argv[1]);
            std::cout << "Stanley gain set to: " << stanley_gain << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Invalid stanley gain argument. Using default value: " << stanley_gain << std::endl;
        }
    } else {
        std::cout << "No stanley gain specified. Using default value: " << stanley_gain << std::endl;
        std::cout << "Usage: ros2 run control control_node <stanley_gain>" << std::endl;
    }
    
    rclcpp::spin(std::make_shared<Control>(stanley_gain));
    rclcpp::shutdown();
    return 0;
}