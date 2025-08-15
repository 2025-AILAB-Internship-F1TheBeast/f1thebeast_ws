/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
#include "control_sim.hpp"
#include <cstring>

void print_usage() {
    std::cout << "Usage: ros2 run control_sim control_sim_node [OPTIONS]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --help                      Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  ros2 run control_sim control_sim_node --ini_file_path <path_to_ini_file>" << std::endl;
}

int main(int argc, char **argv) {

    // 기본 ini 파일 경로
    std::string ini_file_path = "/home/thebeast/Programming/f1thebeast_ws/src/control_ws/control_sim/config/param.ini";
    // 명령행 인자 처리
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0) {
            print_usage();
            return 0;
        } else if (std::strcmp(argv[i], "--ini_file_path") == 0) {
            if (i + 1 < argc) {
                ini_file_path = argv[++i]; // 다음 인자를 ini 파일 경로로 사용
            } else {
                std::cerr << "Error: --ini_file_path option requires a file path argument." << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error: Unknown option '" << argv[i] << "'." << std::endl;
            return 1;
        }
    }

    // ROS2 초기화 및 Control 클래스 인스턴스 생성
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>(ini_file_path));
    rclcpp::shutdown();
    return 0;
}