/*
7월 25일 기준 인지, 판단 모듈이 아직은 없으므로 global path가 있다는 가정하에 차량의 현재 위치와 global waypoint를 비교해서
local path를 구하고 차량을 제어하는 코드를 작성함.
*/
#include "auto_tuning.hpp"
#include <cstring>

void print_usage() {
    std::cout << "Usage: ros2 run auto_tuning auto_tuning_node [OPTIONS]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --stanley-gain-start <value>  Set stanley controller gain start value (default: 1.0)" << std::endl;
    std::cout << "  --stanley-gain-end <value>    Set stanley controller gain end value (default: 5.0)" << std::endl;
    std::cout << "  --stanley-gain-step <value>   Set stanley controller gain step value (default: 0.1)" << std::endl;
    std::cout << "  --lookahead-index-start <value> Set lookahead index start value (default: 0)" << std::endl;
    std::cout << "  --max-lookahead-heading <value> Set maximum lookahead heading value (default: 9)" << std::endl;
    std::cout << "  --max-cte-threshold <value>   Set maximum cross track error threshold (default: 2.0)" << std::endl;
    std::cout << "  --help                        Show this help message" << std::endl;
    std::cout << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "  ros2 run auto_tuning auto_tuning_node --stanley-gain-start 1.0 --stanley-gain-end 5.0 --stanley-gain-step 0.2 --lookahead-index-start 0 --max-lookahead-heading 4 --max-cte-threshold 1.5" << std::endl;
    std::cout << "change the value inside the std::setprecision() to change the number of decimal places." << std::endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    float stanley_gain_start = 1.0f; // 기본값
    float stanley_gain_end = 5.0f;   // 기본값
    float stanley_gain_step = 0.1f;  // 기본값
    int lookahead_idx_start = 0;     // 기본값
    int max_lookahead_heading = 10;   // 기본값 추가
    float max_cte_threshold = 2.0f;  // 기본값
    
    // Command line argument 파싱
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--stanley-gain-start") == 0) {
            if (i + 1 < argc) {
                try {
                    stanley_gain_start = std::stof(argv[i + 1]);
                    std::cout << "Stanley gain start set to: " << stanley_gain_start << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid stanley gain start value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --stanley-gain-start requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--stanley-gain-end") == 0) {
            if (i + 1 < argc) {
                try {
                    stanley_gain_end = std::stof(argv[i + 1]);
                    std::cout << "Stanley gain end set to: " << stanley_gain_end << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid stanley gain end value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --stanley-gain-end requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--stanley-gain-step") == 0) {
            if (i + 1 < argc) {
                try {
                    stanley_gain_step = std::stof(argv[i + 1]);
                    std::cout << "Stanley gain step set to: " << stanley_gain_step << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid stanley gain step value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --stanley-gain-step requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--lookahead-index-start") == 0) {
            if (i + 1 < argc) {
                try {
                    lookahead_idx_start = std::stoi(argv[i + 1]);
                    std::cout << "Lookahead index start set to: " << lookahead_idx_start << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid lookahead index value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --lookahead-index-start requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--max-lookahead-heading") == 0) {
            if (i + 1 < argc) {
                try {
                    max_lookahead_heading = std::stoi(argv[i + 1]);
                    std::cout << "Max lookahead heading set to: " << max_lookahead_heading << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid max lookahead heading value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --max-lookahead-heading requires a value" << std::endl;
                print_usage();
                return 1;
            }
        }
        else if (strcmp(argv[i], "--max-cte-threshold") == 0) {
            if (i + 1 < argc) {
                try {
                    max_cte_threshold = std::stof(argv[i + 1]);
                    std::cout << "Max CTE threshold set to: " << max_cte_threshold << std::endl;
                    i++; // 다음 인수 건너뛰기
                } catch (const std::exception& e) {
                    std::cerr << "Invalid max CTE threshold value: " << argv[i + 1] << std::endl;
                    print_usage();
                    return 1;
                }
            } else {
                std::cerr << "Error: --max-cte-threshold requires a value" << std::endl;
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

    // 값 유효성 검증
    if (stanley_gain_start >= stanley_gain_end) {
        std::cerr << "Error: stanley-gain-start (" << stanley_gain_start 
                  << ") must be less than stanley-gain-end (" << stanley_gain_end << ")" << std::endl;
        return 1;
    }
    
    if (stanley_gain_step <= 0.0f) {
        std::cerr << "Error: stanley-gain-step (" << stanley_gain_step 
                  << ") must be positive" << std::endl;
        return 1;
    }
    
    if (max_cte_threshold <= 0.0f) {
        std::cerr << "Error: max-cte-threshold (" << max_cte_threshold 
                  << ") must be positive" << std::endl;
        return 1;
    }
    
    if (max_lookahead_heading < 0) {
        std::cerr << "Error: max-lookahead-heading (" << max_lookahead_heading 
                  << ") must be non-negative" << std::endl;
        return 1;
    }
    
    if (lookahead_idx_start > max_lookahead_heading) {
        std::cerr << "Error: lookahead-index-start (" << lookahead_idx_start 
                  << ") must be <= max-lookahead-heading (" << max_lookahead_heading << ")" << std::endl;
        return 1;
    }

    // 스텝 수 계산 및 출력
    int total_steps = static_cast<int>(std::ceil((stanley_gain_end - stanley_gain_start) / stanley_gain_step));
    int total_combinations = total_steps * (max_lookahead_heading + 1); // lookahead는 0부터 max_lookahead_heading까지

    std::cout << "Starting auto_tuning_node with:" << std::endl;
    std::cout << "  Stanley gain start: " << stanley_gain_start << std::endl;
    std::cout << "  Stanley gain end: " << stanley_gain_end << std::endl;
    std::cout << "  Stanley gain step: " << stanley_gain_step << std::endl;
    std::cout << "  Lookahead index start: " << lookahead_idx_start << std::endl;
    std::cout << "  Max lookahead heading: " << max_lookahead_heading << std::endl;
    std::cout << "  Max CTE threshold: " << max_cte_threshold << " meters" << std::endl;
    std::cout << "  Total stanley gain range: " << (stanley_gain_end - stanley_gain_start) << std::endl;
    std::cout << "  Estimated stanley gain steps: " << total_steps << std::endl;
    std::cout << "  Lookahead heading combinations: " << (max_lookahead_heading + 1) << std::endl;
    std::cout << "  Total parameter combinations to test: " << total_combinations << std::endl;

    rclcpp::spin(std::make_shared<Auto_tuning>(stanley_gain_start, lookahead_idx_start, stanley_gain_end, stanley_gain_step, max_cte_threshold, max_lookahead_heading));
    rclcpp::shutdown();
    return 0;
}