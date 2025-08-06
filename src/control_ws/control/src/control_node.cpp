/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
#include "control.hpp"
#include <cstring>

void print_usage() {
    std::cout << "Usage: ros2 run control control_node [OPTIONS]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --stanley-gain <value>      Set stanley controller gain (default: 3.5)" << std::endl;
    std::cout << "  --lookahead-heading <value> Set lookahead heading index (default: 3)" << std::endl;
    std::cout << "  --help                      Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  ros2 run control control_node --stanley-gain 9.9 --lookahead-heading 10" << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    float stanley_gain = 3.5f; // 기본값
    int lookahead_heading = 3;  // 기본값
    
    // Command line argument 파싱
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--stanley-gain") == 0) {
            if (i + 1 < argc) {
                try {
                    stanley_gain = std::stof(argv[i + 1]);
                    std::cout << "Stanley gain set to: " << stanley_gain << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid stanley gain value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --stanley-gain requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--lookahead-heading") == 0) {
            if (i + 1 < argc) {
                try {
                    lookahead_heading = std::stoi(argv[i + 1]);
                    std::cout << "Lookahead heading index set to: " << lookahead_heading << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid lookahead heading value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --lookahead-heading requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage();
            return 0;
        }
        else {
            std::cerr << "Unknown argument: " << argv[i] << std::endl;
            print_usage();
            return 1;
        }
    }
    
    std::cout << "Starting control node with:" << std::endl;
    std::cout << "  Stanley gain: " << stanley_gain << std::endl;
    std::cout << "  Lookahead heading index: " << lookahead_heading << std::endl;
    
    rclcpp::spin(std::make_shared<Control>(stanley_gain, lookahead_heading));
    rclcpp::shutdown();
    return 0;
}