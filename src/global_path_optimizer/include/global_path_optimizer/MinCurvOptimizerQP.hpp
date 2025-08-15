#ifndef MINCURV_OPTIMIZER_QP_HPP
#define MINCURV_OPTIMIZER_QP_HPP

#include "MinCurvOptimizer.hpp"
#include <vector>
#include <memory>

// Forward declaration for OSQP
extern "C" {
    typedef struct OSQPWorkspace OSQPWorkspace;
    typedef struct OSQPSettings OSQPSettings;
    typedef struct OSQPData OSQPData;
}

namespace mincurv {

// QP-specific optimization parameters
struct QPParams {
    double eps_abs = 1e-6;          // Absolute tolerance
    double eps_rel = 1e-6;          // Relative tolerance
    int max_iter = 4000;            // Maximum iterations
    bool verbose = false;           // Verbose output
    double alpha = 1.6;             // Relaxation parameter
    double rho = 0.1;               // ADMM step size
    bool adaptive_rho = true;       // Adaptive rho
    bool polish = true;             // Polish solution
};

class MinCurvOptimizerQP : public MinCurvOptimizer {
private:
    QPParams qp_params_;
    
    // QP problem formulation
    struct QPProblem {
        std::vector<double> P_data;     // Objective matrix (sparse)
        std::vector<int> P_indices;     // Row indices
        std::vector<int> P_indptr;      // Column pointers
        std::vector<double> q;          // Linear objective vector
        
        std::vector<double> A_data;     // Constraint matrix (sparse)
        std::vector<int> A_indices;     // Row indices
        std::vector<int> A_indptr;      // Column pointers
        std::vector<double> l;          // Lower bounds
        std::vector<double> u;          // Upper bounds
        
        int n;                          // Number of variables
        int m;                          // Number of constraints
    };

public:
    MinCurvOptimizerQP(const OptParams& opt_params, 
                       const VehParams& veh_params,
                       const OptimOpts& optim_opts,
                       const QPParams& qp_params = QPParams(),
                       bool debug = false);
    
    // Override optimization method to use QP
    std::vector<double> optimizeMinCurvature() override;
    
private:
    // QP problem formulation methods
    QPProblem formulateQPProblem(const std::vector<Point2D>& centerline,
                                 const std::vector<TrackWidth>& track_widths,
                                 const std::vector<Point2D>& normal_vectors) const;
    
    // Build curvature approximation matrices
    void buildCurvatureMatrix(const std::vector<Point2D>& centerline,
                             const std::vector<Point2D>& normal_vectors,
                             std::vector<double>& P_data,
                             std::vector<int>& P_indices,
                             std::vector<int>& P_indptr) const;
    
    // Build constraint matrices (track boundaries)
    void buildConstraintMatrix(const std::vector<TrackWidth>& track_widths,
                              std::vector<double>& A_data,
                              std::vector<int>& A_indices,
                              std::vector<int>& A_indptr,
                              std::vector<double>& l,
                              std::vector<double>& u) const;
    
    // Solve QP problem using OSQP
    std::vector<double> solveQP(const QPProblem& problem) const;
    
    // Utility functions for sparse matrix operations
    void addToSparseMatrix(std::vector<double>& data,
                          std::vector<int>& indices,
                          std::vector<int>& indptr,
                          int row, int col, double value) const;
};

// Alternative implementation using Eigen + simple QP solver
class MinCurvOptimizerEigen : public MinCurvOptimizer {
private:
    double regularization_weight_ = 1e-6;
    
public:
    MinCurvOptimizerEigen(const OptParams& opt_params, 
                         const VehParams& veh_params,
                         const OptimOpts& optim_opts,
                         bool debug = false);
    
    // Override optimization method to use Eigen-based QP
    std::vector<double> optimizeMinCurvature() override;
    
private:
    // Simple QP solver using Eigen
    std::vector<double> solveQPEigen(const std::vector<std::vector<double>>& H,
                                    const std::vector<double>& f,
                                    const std::vector<std::vector<double>>& A,
                                    const std::vector<double>& b_lower,
                                    const std::vector<double>& b_upper) const;
    
    // Build problem matrices using Eigen
    void buildQPMatrices(const std::vector<Point2D>& centerline,
                        const std::vector<Point2D>& normal_vectors,
                        const std::vector<TrackWidth>& track_widths,
                        std::vector<std::vector<double>>& H,
                        std::vector<double>& f,
                        std::vector<std::vector<double>>& A,
                        std::vector<double>& b_lower,
                        std::vector<double>& b_upper) const;
};

} // namespace mincurv

#endif // MINCURV_OPTIMIZER_QP_HPP
