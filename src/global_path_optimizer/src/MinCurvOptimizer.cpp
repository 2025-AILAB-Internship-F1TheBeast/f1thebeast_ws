#include "global_path_optimizer/MinCurvOptimizer.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <regex>
#include <iomanip>

namespace mincurv {

// Point2D implementation
double Point2D::norm() const {
    return std::sqrt(x * x + y * y);
}

Point2D Point2D::normalized() const {
    double n = norm();
    if (n < 1e-10) return Point2D(0, 0);
    return Point2D(x / n, y / n);
}

// MinCurvOptimizer implementation
MinCurvOptimizer::MinCurvOptimizer(const OptParams& opt_params, 
                                   const VehParams& veh_params,
                                   const OptimOpts& optim_opts,
                                   bool debug)
    : opt_params_(opt_params), veh_params_(veh_params), 
      optim_opts_(optim_opts), debug_(debug) {
}

bool MinCurvOptimizer::loadTrackData(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open track file: " << file_path << std::endl;
        return false;
    }
    
    std::string line;
    std::getline(file, line); // Skip header
    
    centerline_.clear();
    track_widths_.clear();
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        auto values = split(line, ',');
        if (values.size() < 4) continue;
        
        double x = std::stod(trim(values[0]));
        double y = std::stod(trim(values[1]));
        double w_right = std::stod(trim(values[2]));
        double w_left = std::stod(trim(values[3]));
        
        centerline_.emplace_back(x, y);
        track_widths_.emplace_back(w_right, w_left);
    }
    
    file.close();
    
    if (debug_) {
        std::cout << "INFO: Loaded track with " << centerline_.size() << " points" << std::endl;
    }
    
    return !centerline_.empty();
}

void MinCurvOptimizer::calculateNormalVectors() {
    size_t N = centerline_.size();
    normal_vectors_.resize(N);
    
    for (size_t i = 0; i < N; ++i) {
        Point2D tangent;
        
        if (i == 0) {
            // Forward difference for first point
            tangent = centerline_[i + 1] - centerline_[i];
        } else if (i == N - 1) {
            // Backward difference for last point (connect to first)
            tangent = centerline_[0] - centerline_[i - 1];
        } else {
            // Central difference for middle points
            tangent = centerline_[i + 1] - centerline_[i - 1];
        }
        
        tangent = tangent.normalized();
        
        // Calculate normal vector (rotate tangent by 90 degrees)
        normal_vectors_[i] = Point2D(-tangent.y, tangent.x);
    }
}

std::vector<double> MinCurvOptimizer::calculateCurvatureFiniteDiff(const std::vector<Point2D>& points) const {
    size_t N = points.size();
    std::vector<double> curvature(N, 0.0);
    
    for (size_t i = 0; i < N; ++i) {
        // Get three consecutive points
        Point2D p1 = points[(i - 1 + N) % N];
        Point2D p2 = points[i];
        Point2D p3 = points[(i + 1) % N];
        
        // Calculate first derivatives
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;
        
        // Calculate second derivatives
        double d2x = dx2 - dx1;
        double d2y = dy2 - dy1;
        
        // Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        double denominator = std::pow(dx1 * dx1 + dy1 * dy1, 1.5);
        if (denominator > 1e-10) {
            curvature[i] = std::abs(dx1 * d2y - dy1 * d2x) / denominator;
        } else {
            curvature[i] = 0.0;
        }
    }
    
    return curvature;
}

void MinCurvOptimizer::resampleTrack(double target_stepsize) {
    if (centerline_.empty()) return;
    
    // Calculate cumulative distances
    std::vector<double> distances(centerline_.size(), 0.0);
    for (size_t i = 1; i < centerline_.size(); ++i) {
        Point2D diff = centerline_[i] - centerline_[i - 1];
        distances[i] = distances[i - 1] + diff.norm();
    }
    
    double total_length = distances.back();
    size_t n_points = static_cast<size_t>(total_length / target_stepsize);
    
    // Create new distance array
    std::vector<double> new_distances(n_points);
    for (size_t i = 0; i < n_points; ++i) {
        new_distances[i] = (total_length * i) / (n_points - 1);
    }
    
    // Interpolate points and widths
    std::vector<Point2D> new_centerline(n_points);
    std::vector<TrackWidth> new_track_widths(n_points);
    
    for (size_t i = 0; i < n_points; ++i) {
        double target_dist = new_distances[i];
        
        // Find interpolation indices
        auto it = std::lower_bound(distances.begin(), distances.end(), target_dist);
        size_t idx2 = std::distance(distances.begin(), it);
        size_t idx1 = (idx2 > 0) ? idx2 - 1 : 0;
        
        if (idx2 >= distances.size()) idx2 = distances.size() - 1;
        if (idx1 == idx2) {
            new_centerline[i] = centerline_[idx1];
            new_track_widths[i] = track_widths_[idx1];
        } else {
            double t = (target_dist - distances[idx1]) / (distances[idx2] - distances[idx1]);
            new_centerline[i].x = centerline_[idx1].x + t * (centerline_[idx2].x - centerline_[idx1].x);
            new_centerline[i].y = centerline_[idx1].y + t * (centerline_[idx2].y - centerline_[idx1].y);
            new_track_widths[i].right = track_widths_[idx1].right + t * (track_widths_[idx2].right - track_widths_[idx1].right);
            new_track_widths[i].left = track_widths_[idx1].left + t * (track_widths_[idx2].left - track_widths_[idx1].left);
        }
    }
    
    centerline_ = std::move(new_centerline);
    track_widths_ = std::move(new_track_widths);
    distances_ = std::move(new_distances);
    
    if (debug_) {
        std::cout << "INFO: Resampled to " << centerline_.size() << " points" << std::endl;
    }
}

std::vector<double> MinCurvOptimizer::optimizeMinCurvature() {
    size_t N = centerline_.size();
    std::vector<double> alpha(N, 0.0);  // lateral deviations
    
    // Calculate track constraints
    std::vector<double> max_right(N), max_left(N);
    for (size_t i = 0; i < N; ++i) {
        max_right[i] = std::min(track_widths_[i].right - 1.0, optim_opts_.width_opt / 2);
        max_left[i] = std::min(track_widths_[i].left - 1.0, optim_opts_.width_opt / 2);
    }
    
    // Iterative optimization
    for (int iteration = 0; iteration < opt_params_.max_iterations; ++iteration) {
        std::vector<double> alpha_old = alpha;
        
        // Calculate current path
        std::vector<Point2D> current_path(N);
        for (size_t i = 0; i < N; ++i) {
            current_path[i] = centerline_[i] + normal_vectors_[i] * alpha[i];
        }
        
        // Calculate current curvature
        auto current_curvature = calculateCurvatureFiniteDiff(current_path);
        
        // Simple gradient descent approach
        double learning_rate = 0.1;
        
        for (size_t i = 0; i < N; ++i) {
            // Calculate gradient of curvature w.r.t. alpha[i]
            double eps = 0.01;
            
            // Test positive perturbation
            std::vector<double> alpha_test = alpha;
            alpha_test[i] += eps;
            std::vector<Point2D> test_path(N);
            for (size_t j = 0; j < N; ++j) {
                test_path[j] = centerline_[j] + normal_vectors_[j] * alpha_test[j];
            }
            auto curvature_plus = calculateCurvatureFiniteDiff(test_path);
            
            // Test negative perturbation
            alpha_test = alpha;
            alpha_test[i] -= eps;
            for (size_t j = 0; j < N; ++j) {
                test_path[j] = centerline_[j] + normal_vectors_[j] * alpha_test[j];
            }
            auto curvature_minus = calculateCurvatureFiniteDiff(test_path);
            
            // Calculate objective function values
            double obj_plus = 0.0, obj_minus = 0.0;
            for (size_t j = 0; j < N; ++j) {
                obj_plus += curvature_plus[j] * curvature_plus[j];
                obj_minus += curvature_minus[j] * curvature_minus[j];
            }
            
            // Approximate gradient
            double grad = (obj_plus - obj_minus) / (2 * eps);
            
            // Update alpha
            alpha[i] -= learning_rate * grad;
            
            // Apply constraints
            alpha[i] = std::max(-max_left[i], std::min(max_right[i], alpha[i]));
            
            // Apply curvature constraint (simple penalty)
            if (current_curvature[i] > veh_params_.curvlim) {
                alpha[i] *= 0.9;  // Reduce deviation to lower curvature
            }
        }
        
        // Check convergence
        double change = 0.0;
        for (size_t i = 0; i < N; ++i) {
            double diff = alpha[i] - alpha_old[i];
            change += diff * diff;
        }
        change = std::sqrt(change);
        
        if (change < opt_params_.convergence_threshold) {
            if (debug_) {
                std::cout << "INFO: Converged after " << iteration + 1 << " iterations" << std::endl;
            }
            break;
        }
        
        if (iteration % 20 == 0 && debug_) {
            double max_curv = *std::max_element(current_curvature.begin(), current_curvature.end());
            double avg_curv = 0.0;
            for (double c : current_curvature) avg_curv += c;
            avg_curv /= current_curvature.size();
            std::cout << "Iteration " << iteration << ": max_curvature=" << max_curv 
                     << ", avg_curvature=" << avg_curv << std::endl;
        }
    }
    
    return alpha;
}

std::vector<Point2D> MinCurvOptimizer::getOptimalPath(const std::vector<double>& alpha) const {
    std::vector<Point2D> optimal_path(centerline_.size());
    for (size_t i = 0; i < centerline_.size(); ++i) {
        optimal_path[i] = centerline_[i] + normal_vectors_[i] * alpha[i];
    }
    return optimal_path;
}

bool MinCurvOptimizer::exportToCSV(const std::string& file_path, const std::vector<Point2D>& optimal_path) const {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to create output file: " << file_path << std::endl;
        return false;
    }
    
    file << "s_m,x_m,y_m\n";
    for (size_t i = 0; i < optimal_path.size() && i < distances_.size(); ++i) {
        file << std::fixed << std::setprecision(6) 
             << distances_[i] << "," 
             << optimal_path[i].x << "," 
             << optimal_path[i].y << "\n";
    }
    
    file.close();
    return true;
}

void MinCurvOptimizer::plotResults(const std::vector<Point2D>& optimal_path, 
                                   const std::vector<double>& curvature,
                                   const std::string& track_name,
                                   const std::string& save_path) const {
    // Note: This is a placeholder for plotting functionality
    // In a real implementation, you would use a plotting library like matplotlib-cpp, 
    // gnuplot-iostream, or similar
    
    if (debug_) {
        std::cout << "INFO: Plotting functionality not implemented in this C++ version" << std::endl;
        std::cout << "INFO: Consider using Python plotting or integrating a C++ plotting library" << std::endl;
    }
}

// Utility functions
std::vector<std::string> MinCurvOptimizer::split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    
    return tokens;
}

std::string MinCurvOptimizer::trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

// ConfigParser implementation
bool ConfigParser::loadFromFile(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "ERROR: Failed to open config file: " << file_path << std::endl;
        return false;
    }
    
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        line = MinCurvOptimizer::trim(line);
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        
        // Check for section header
        if (line[0] == '[' && line.back() == ']') {
            current_section = line.substr(1, line.length() - 2);
            continue;
        }
        
        // Parse key-value pair
        size_t eq_pos = line.find('=');
        if (eq_pos != std::string::npos) {
            std::string key = MinCurvOptimizer::trim(line.substr(0, eq_pos));
            std::string value = MinCurvOptimizer::trim(line.substr(eq_pos + 1));
            sections_[current_section][key] = value;
        }
    }
    
    file.close();
    return true;
}

std::string ConfigParser::getValue(const std::string& section, const std::string& key) const {
    auto section_it = sections_.find(section);
    if (section_it != sections_.end()) {
        auto key_it = section_it->second.find(key);
        if (key_it != section_it->second.end()) {
            return key_it->second;
        }
    }
    return "";
}

OptParams ConfigParser::parseOptParams(const std::string& json_str) const {
    OptParams params;
    
    // Initialize with default values
    params.stepsize = 3.0;
    params.max_iterations = 100;
    params.convergence_threshold = 1e-6;
    params.alpha_max = 5.0;
    
    // Simple JSON-like parsing (for demonstration)
    // In a real implementation, use a proper JSON library like nlohmann/json
    std::regex stepsize_regex(R"("stepsize"\s*:\s*([0-9.]+))");
    std::regex max_iter_regex(R"("max_iterations"\s*:\s*([0-9]+))");
    std::regex conv_thresh_regex(R"("convergence_threshold"\s*:\s*([0-9.e-]+))");
    std::regex alpha_max_regex(R"("alpha_max"\s*:\s*([0-9.]+))");
    
    std::smatch match;
    if (std::regex_search(json_str, match, stepsize_regex)) {
        params.stepsize = std::stod(match[1]);
    }
    if (std::regex_search(json_str, match, max_iter_regex)) {
        params.max_iterations = std::stoi(match[1]);
    }
    if (std::regex_search(json_str, match, conv_thresh_regex)) {
        params.convergence_threshold = std::stod(match[1]);
    }
    if (std::regex_search(json_str, match, alpha_max_regex)) {
        params.alpha_max = std::stod(match[1]);
    }
    
    return params;
}

VehParams ConfigParser::parseVehParams(const std::string& json_str) const {
    VehParams params;
    
    // Initialize with default values
    params.width = 2.0;
    params.curvlim = 0.12;
    
    std::regex width_regex(R"("width"\s*:\s*([0-9.]+))");
    std::regex curvlim_regex(R"("curvlim"\s*:\s*([0-9.]+))");
    
    std::smatch match;
    if (std::regex_search(json_str, match, width_regex)) {
        params.width = std::stod(match[1]);
    }
    if (std::regex_search(json_str, match, curvlim_regex)) {
        params.curvlim = std::stod(match[1]);
    }
    
    return params;
}

OptimOpts ConfigParser::parseOptimOpts(const std::string& json_str) const {
    OptimOpts params;
    
    // Initialize with default values
    params.width_opt = 3.4;
    params.iqp_iters_min = 3;
    params.iqp_curverror_allowed = 0.01;
    
    std::regex width_opt_regex(R"("width_opt"\s*:\s*([0-9.]+))");
    std::regex iqp_iters_regex(R"("iqp_iters_min"\s*:\s*([0-9]+))");
    std::regex iqp_curverror_regex(R"("iqp_curverror_allowed"\s*:\s*([0-9.]+))");
    
    std::smatch match;
    if (std::regex_search(json_str, match, width_opt_regex)) {
        params.width_opt = std::stod(match[1]);
    }
    if (std::regex_search(json_str, match, iqp_iters_regex)) {
        params.iqp_iters_min = std::stoi(match[1]);
    }
    if (std::regex_search(json_str, match, iqp_curverror_regex)) {
        params.iqp_curverror_allowed = std::stod(match[1]);
    }
    
    return params;
}

} // namespace mincurv
