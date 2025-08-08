#include <visualize_global_path.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizeGlobal>());
    rclcpp::shutdown();
    return 0;
}