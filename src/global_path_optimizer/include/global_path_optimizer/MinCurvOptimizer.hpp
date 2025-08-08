#ifndef MINCURV_OPTIMIZER_HPP
#define MINCURV_OPTIMIZER_HPP

#include <vector>
#include <string>
#include <memory>
#include <map>

namespace mincurv {

// 2D Point structure
struct Point2D {
    double x, y;
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_val, double y_val) : x(x_val), y(y_val) {}
    
    Point2D operator+(const Point2D& other) const {
        return Point2D(x + other.x, y + other.y);
    }
    
    Point2D operator-(const Point2D& other) const {
        return Point2D(x - other.x, y - other.y);
    }
    
    Point2D operator*(double scalar) const {
        return Point2D(x * scalar, y * scalar);
    }
    
    double norm() const;
    Point2D normalized() const;
};

// Track width information
struct TrackWidth {
    double right, left;
    TrackWidth() : right(0.0), left(0.0) {}
    TrackWidth(double r, double l) : right(r), left(l) {}
};

// Optimization parameters
struct OptParams {
    double stepsize;
    int max_iterations;
    double convergence_threshold;
    double alpha_max;
};

// Vehicle parameters
struct VehParams {
    double width;
    double curvlim;
};

// Optimization options
struct OptimOpts {
    double width_opt;
    int iqp_iters_min;
    double iqp_curverror_allowed;
};

// Main optimizer class
class MinCurvOptimizer {
protected:
    OptParams opt_params_;
    VehParams veh_params_;
    OptimOpts optim_opts_;
    bool debug_;
    
    std::vector<Point2D> centerline_;
    std::vector<TrackWidth> track_widths_;
    std::vector<Point2D> normal_vectors_;
    std::vector<double> distances_;
    
public:
    MinCurvOptimizer(const OptParams& opt_params, 
                     const VehParams& veh_params,
                     const OptimOpts& optim_opts,
                     bool debug = false);
    
    // Load track data from CSV file
    bool loadTrackData(const std::string& file_path);
    
    // Calculate normal vectors for the centerline
    void calculateNormalVectors();
    
    // Calculate curvature using finite differences
    std::vector<double> calculateCurvatureFiniteDiff(const std::vector<Point2D>& points) const;
    
    // Resample track to uniform spacing
    void resampleTrack(double target_stepsize);
    
    // Main optimization function
    virtual std::vector<double> optimizeMinCurvature();
    
    // Get optimal path
    std::vector<Point2D> getOptimalPath(const std::vector<double>& alpha) const;
    
    // Export results to CSV
    bool exportToCSV(const std::string& file_path, const std::vector<Point2D>& optimal_path) const;
    
    // Plot results (if visualization library is available)
    void plotResults(const std::vector<Point2D>& optimal_path, 
                     const std::vector<double>& curvature,
                     const std::string& track_name,
                     const std::string& save_path = "") const;
    
    // Getters
    const std::vector<Point2D>& getCenterline() const { return centerline_; }
    const std::vector<TrackWidth>& getTrackWidths() const { return track_widths_; }
    const std::vector<Point2D>& getNormalVectors() const { return normal_vectors_; }
    const std::vector<double>& getDistances() const { return distances_; }
    
    // Utility functions
    static std::vector<std::string> split(const std::string& str, char delimiter);
    static std::string trim(const std::string& str);
};

// Configuration parser class
class ConfigParser {
private:
    std::map<std::string, std::map<std::string, std::string>> sections_;
    
public:
    bool loadFromFile(const std::string& file_path);
    std::string getValue(const std::string& section, const std::string& key) const;
    
    // Parse JSON-like string to extract parameters
    OptParams parseOptParams(const std::string& json_str) const;
    VehParams parseVehParams(const std::string& json_str) const;
    OptimOpts parseOptimOpts(const std::string& json_str) const;
};

} // namespace mincurv

#endif // MINCURV_OPTIMIZER_HPP
