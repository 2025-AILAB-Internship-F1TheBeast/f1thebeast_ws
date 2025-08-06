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
    std::cout << "  --velocity-gain <value>     Set velocity gain (default: 1.0)" << std::endl;
    std::cout << "  --enable-metrics            Enable metrics recording" << std::endl;
    std::cout << "  --help                      Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  ros2 run control control_node --stanley-gain 3.5 --velocity-gain 1.0 --enable-metrics" << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    float stanley_gain = 3.5f; // 기본값
    float velocity_gain = 1.0f; // 기본값
    bool enable_metrics = false; // 기본값: metrics 비활성화
    
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
        else if (strcmp(argv[i], "--velocity-gain") == 0) {
            if (i + 1 < argc) {
                try {
                    velocity_gain = std::stof(argv[i + 1]);
                    std::cout << "Velocity gain set to: " << velocity_gain << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid velocity gain value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --velocity-gain requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--enable-metrics") == 0) {
            enable_metrics = true;
            std::cout << "Metrics recording enabled" << std::endl;
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
    std::cout << "  Velocity gain: " << velocity_gain << std::endl;
    std::cout << "  Metrics recording: " << (enable_metrics ? "enabled" : "disabled") << std::endl;

    rclcpp::spin(std::make_shared<Control>(stanley_gain, enable_metrics));
    rclcpp::shutdown();
    return 0;
}