#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/string.hpp"
#include "global_path_optimizer/MinCurvOptimizer.hpp"
#include "global_path_optimizer/MinCurvOptimizerQP.hpp"
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <memory>

class PathOptimizerNode : public rclcpp::Node
{
public:
    PathOptimizerNode() : Node("path_optimizer_node")
    {
        // Publisher for optimal path
        path_publisher_ = this->create_publisher<geometry_msgs::msg::Polygon>("optimal_path", 10);
        
        // Publisher for status messages
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("optimizer_status", 10);
        
        // Timer to run optimization once
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathOptimizerNode::run_optimization, this));
            
        // Package path setup
        package_path_ = "/home/yongwoo/f1thebeast_ws/src/global_path_optimizer";
        track_name_ = "berlin_2018";
        track_file_ = package_path_ + "/inputs/tracks/" + track_name_ + ".csv";
        config_file_ = package_path_ + "/params/onlymincurv.ini";
        
        // Create outputs directory
        std::filesystem::create_directories(package_path_ + "/outputs");
        
        RCLCPP_INFO(this->get_logger(), "Path Optimizer Node initialized");
        RCLCPP_INFO(this->get_logger(), "Track file: %s", track_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());
    }

private:
    void run_optimization()
    {
        // Run optimization only once
        timer_->cancel();
        
        using namespace mincurv;
        
        RCLCPP_INFO(this->get_logger(), "=============================================================");
        RCLCPP_INFO(this->get_logger(), "COMPARISON: Gradient Descent vs QP-based Optimization");
        RCLCPP_INFO(this->get_logger(), "=============================================================");
        
        // Load configuration
        ConfigParser config_parser;
        if (!config_parser.loadFromFile(config_file_)) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to load configuration file: %s", config_file_.c_str());
            return;
        }
        
        auto opt_params = config_parser.parseOptParams(
            config_parser.getValue("GENERAL_OPTIONS", "opt_params"));
        auto veh_params = config_parser.parseVehParams(
            config_parser.getValue("GENERAL_OPTIONS", "veh_params"));
        auto optim_opts = config_parser.parseOptimOpts(
            config_parser.getValue("OPTIMIZATION_OPTIONS", "optim_opts_mincurv"));
        
        // Test both optimizers
        std::vector<std::pair<std::string, std::unique_ptr<MinCurvOptimizer>>> optimizers;
        
        optimizers.emplace_back("Gradient Descent", 
            std::make_unique<MinCurvOptimizer>(opt_params, veh_params, optim_opts, true));
        optimizers.emplace_back("QP (Eigen-based)", 
            std::make_unique<MinCurvOptimizerQP>(opt_params, veh_params, optim_opts, mincurv::QPParams(), true));
        
        std::vector<std::string> output_files = {
            package_path_ + "/outputs/global_path_gradient_descent.csv",
            package_path_ + "/outputs/global_path_qp_eigen.csv"
        };
        
        for (size_t opt_idx = 0; opt_idx < optimizers.size(); ++opt_idx) {
            const auto& [name, optimizer] = optimizers[opt_idx];
            
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Testing: %s", name.c_str());
            RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
            
            // Publish status
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "Starting optimization with " + name;
            status_publisher_->publish(status_msg);
            
            // Start timing
            auto t_start = std::chrono::high_resolution_clock::now();
            
            // Load track data
            RCLCPP_INFO(this->get_logger(), "INFO: Loading track data...");
            if (!optimizer->loadTrackData(track_file_)) {
                RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to load track data!");
                continue;
            }
            
            const auto& centerline = optimizer->getCenterline();
            const auto& track_widths = optimizer->getTrackWidths();
            
            RCLCPP_INFO(this->get_logger(), "INFO: Track has %zu points", centerline.size());
            
            // Resample track
            RCLCPP_INFO(this->get_logger(), "INFO: Resampling track...");
            optimizer->resampleTrack(opt_params.stepsize);
            
            // Calculate normal vectors
            RCLCPP_INFO(this->get_logger(), "INFO: Calculating normal vectors...");
            optimizer->calculateNormalVectors();
            
            // Perform optimization
            RCLCPP_INFO(this->get_logger(), "INFO: Starting optimization...");
            auto alpha_optimal = optimizer->optimizeMinCurvature();
            
            // Calculate optimal path
            auto optimal_path = optimizer->getOptimalPath(alpha_optimal);
            
            // Calculate final curvature
            auto final_curvature = optimizer->calculateCurvatureFiniteDiff(optimal_path);
            
            // End timing
            auto t_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
            
            // Calculate statistics
            double max_curvature = *std::max_element(final_curvature.begin(), final_curvature.end());
            double avg_curvature = 0.0;
            for (double c : final_curvature) avg_curvature += c;
            avg_curvature /= final_curvature.size();
            
            double max_deviation = 0.0;
            for (double alpha : alpha_optimal) {
                max_deviation = std::max(max_deviation, std::abs(alpha));
            }
            
            RCLCPP_INFO(this->get_logger(), "INFO: Optimization completed in %.2fs", duration.count() / 1000.0);
            RCLCPP_INFO(this->get_logger(), "INFO: Maximum curvature: %.4f rad/m", max_curvature);
            RCLCPP_INFO(this->get_logger(), "INFO: Average curvature: %.4f rad/m", avg_curvature);
            RCLCPP_INFO(this->get_logger(), "INFO: Maximum lateral deviation: %.2f m", max_deviation);
            
            // Export to CSV
            if (optimizer->exportToCSV(output_files[opt_idx], optimal_path)) {
                RCLCPP_INFO(this->get_logger(), "INFO: Path exported to: %s", output_files[opt_idx].c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "ERROR: Failed to export path!");
            }
            
            // Publish optimal path as ROS2 message
            publish_path(optimal_path, name);
            
            // Summary
            const auto& distances = optimizer->getDistances();
            double total_length = distances.empty() ? 0.0 : distances.back();
            
            RCLCPP_INFO(this->get_logger(), "SUMMARY - %s:", name.c_str());
            RCLCPP_INFO(this->get_logger(), "- Total path length: %.2f m", total_length);
            RCLCPP_INFO(this->get_logger(), "- Max curvature: %.4f rad/m", max_curvature);
            RCLCPP_INFO(this->get_logger(), "- Max deviation: %.2f m", max_deviation);
            RCLCPP_INFO(this->get_logger(), "- Computation time: %.2fs", duration.count() / 1000.0);
        }
        
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "COMPARISON COMPLETED");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        
        // Final status message
        auto final_status = std_msgs::msg::String();
        final_status.data = "Optimization comparison completed successfully";
        status_publisher_->publish(final_status);
    }
    
    void publish_path(const std::vector<mincurv::Point2D>& path, const std::string& optimizer_name)
    {
        auto path_msg = geometry_msgs::msg::Polygon();
        
        for (const auto& point : path) {
            geometry_msgs::msg::Point32 ros_point;
            ros_point.x = static_cast<float>(point.x);
            ros_point.y = static_cast<float>(point.y);
            ros_point.z = 0.0f;
            path_msg.points.push_back(ros_point);
        }
        
        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published optimal path for %s (%zu points)", 
                   optimizer_name.c_str(), path.size());
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::string package_path_;
    std::string track_name_;
    std::string track_file_;
    std::string config_file_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathOptimizerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
