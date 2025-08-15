#include "global_path_optimizer/MinCurvOptimizerQP.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace mincurv {

// Simple matrix operations for QP solving
class SimpleMatrix {
public:
    std::vector<std::vector<double>> data;
    size_t rows, cols;
    
    SimpleMatrix(size_t r, size_t c) : rows(r), cols(c) {
        data.resize(r, std::vector<double>(c, 0.0));
    }
    
    double& operator()(size_t i, size_t j) { return data[i][j]; }
    const double& operator()(size_t i, size_t j) const { return data[i][j]; }
    
    // Matrix-vector multiplication
    std::vector<double> operator*(const std::vector<double>& vec) const {
        assert(vec.size() == cols);
        std::vector<double> result(rows, 0.0);
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result[i] += data[i][j] * vec[j];
            }
        }
        return result;
    }
    
    // Transpose
    SimpleMatrix transpose() const {
        SimpleMatrix result(cols, rows);
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result(j, i) = data[i][j];
            }
        }
        return result;
    }
    
    // Add regularization to diagonal
    void addDiagonal(double value) {
        for (size_t i = 0; i < std::min(rows, cols); ++i) {
            data[i][i] += value;
        }
    }
};

// Eigen-based QP optimizer implementation
MinCurvOptimizerEigen::MinCurvOptimizerEigen(const OptParams& opt_params, 
                                           const VehParams& veh_params,
                                           const OptimOpts& optim_opts,
                                           bool debug)
    : MinCurvOptimizer(opt_params, veh_params, optim_opts, debug) {
}

std::vector<double> MinCurvOptimizerEigen::optimizeMinCurvature() {
    size_t N = centerline_.size();
    
    if (debug_) {
        std::cout << "INFO: Starting QP-based minimum curvature optimization..." << std::endl;
    }
    
    // Build QP problem matrices
    std::vector<std::vector<double>> H;
    std::vector<double> f;
    std::vector<std::vector<double>> A;
    std::vector<double> b_lower, b_upper;
    
    buildQPMatrices(centerline_, normal_vectors_, track_widths_, H, f, A, b_lower, b_upper);
    
    // Solve QP problem
    auto alpha = solveQPEigen(H, f, A, b_lower, b_upper);
    
    if (debug_) {
        std::cout << "INFO: QP optimization completed" << std::endl;
    }
    
    return alpha;
}

void MinCurvOptimizerEigen::buildQPMatrices(const std::vector<Point2D>& centerline,
                                          const std::vector<Point2D>& normal_vectors,
                                          const std::vector<TrackWidth>& track_widths,
                                          std::vector<std::vector<double>>& H,
                                          std::vector<double>& f,
                                          std::vector<std::vector<double>>& A,
                                          std::vector<double>& b_lower,
                                          std::vector<double>& b_upper) const {
    
    size_t N = centerline.size();
    
    // Initialize matrices
    H.assign(N, std::vector<double>(N, 0.0));
    f.assign(N, 0.0);
    
    // Build objective function: minimize sum of squared curvatures
    // This is an approximation of the curvature as a quadratic function of alpha
    
    // Simple finite difference approximation for curvature
    // κ ≈ |α_{i+1} - 2α_i + α_{i-1}| / h²
    // Objective: minimize Σ (α_{i+1} - 2α_i + α_{i-1})²
    
    double weight = 1.0;  // Curvature minimization weight
    
    for (size_t i = 0; i < N; ++i) {
        size_t i_prev = (i - 1 + N) % N;
        size_t i_next = (i + 1) % N;
        
        // Second derivative approximation: α_{i+1} - 2α_i + α_{i-1}
        // Contribution to quadratic form: weight * (α_{i+1} - 2α_i + α_{i-1})²
        
        // H matrix entries for quadratic term
        H[i_prev][i_prev] += weight;        // α_{i-1}²
        H[i][i] += 4.0 * weight;            // 4α_i²
        H[i_next][i_next] += weight;        // α_{i+1}²
        H[i_prev][i] -= 2.0 * weight;       // -2α_{i-1}α_i
        H[i][i_prev] -= 2.0 * weight;       // -2α_iα_{i-1}
        H[i][i_next] -= 2.0 * weight;       // -2α_iα_{i+1}
        H[i_next][i] -= 2.0 * weight;       // -2α_{i+1}α_i
        H[i_prev][i_next] += 2.0 * weight;  // 2α_{i-1}α_{i+1}
        H[i_next][i_prev] += 2.0 * weight;  // 2α_{i+1}α_{i-1}
    }
    
    // Add regularization to make problem well-conditioned
    for (size_t i = 0; i < N; ++i) {
        H[i][i] += regularization_weight_;
    }
    
    // Linear term f is zero for pure curvature minimization
    // (could add terms for staying close to centerline)
    
    // Constraint matrix A for track boundaries
    // Simple box constraints: -w_left ≤ α_i ≤ w_right
    A.assign(N, std::vector<double>(N, 0.0));
    b_lower.resize(N);
    b_upper.resize(N);
    
    for (size_t i = 0; i < N; ++i) {
        A[i][i] = 1.0;  // Identity matrix for box constraints
        
        // Apply safety margin and optimization width constraints
        double max_right = std::min(track_widths[i].right - 1.0, optim_opts_.width_opt / 2);
        double max_left = std::min(track_widths[i].left - 1.0, optim_opts_.width_opt / 2);
        
        b_lower[i] = -max_left;
        b_upper[i] = max_right;
    }
}

std::vector<double> MinCurvOptimizerEigen::solveQPEigen(const std::vector<std::vector<double>>& H,
                                                       const std::vector<double>& f,
                                                       const std::vector<std::vector<double>>& A,
                                                       const std::vector<double>& b_lower,
                                                       const std::vector<double>& b_upper) const {
    
    size_t N = H.size();
    
    // For box constraints with identity matrix A, we can use projected gradient descent
    // This is a simple QP solver for problems of the form:
    // minimize: 0.5 * x^T * H * x + f^T * x
    // subject to: b_lower ≤ x ≤ b_upper
    
    std::vector<double> x(N, 0.0);  // Initial solution
    std::vector<double> gradient(N);
    
    double learning_rate = 0.01;
    int max_iterations = 1000;
    double tolerance = 1e-6;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Compute gradient: grad = H * x + f
        for (size_t i = 0; i < N; ++i) {
            gradient[i] = f[i];
            for (size_t j = 0; j < N; ++j) {
                gradient[i] += H[i][j] * x[j];
            }
        }
        
        // Gradient descent step
        std::vector<double> x_new(N);
        for (size_t i = 0; i < N; ++i) {
            x_new[i] = x[i] - learning_rate * gradient[i];
            
            // Project onto constraints
            x_new[i] = std::max(b_lower[i], std::min(b_upper[i], x_new[i]));
        }
        
        // Check convergence
        double change = 0.0;
        for (size_t i = 0; i < N; ++i) {
            double diff = x_new[i] - x[i];
            change += diff * diff;
        }
        change = std::sqrt(change);
        
        x = x_new;
        
        if (change < tolerance) {
            if (debug_) {
                std::cout << "INFO: QP solver converged after " << iter + 1 << " iterations" << std::endl;
            }
            break;
        }
        
        if (iter % 100 == 0 && debug_) {
            std::cout << "QP iteration " << iter << ": change=" << change << std::endl;
        }
    }
    
    return x;
}

// OSQP-based implementation (placeholder - requires OSQP library)
MinCurvOptimizerQP::MinCurvOptimizerQP(const OptParams& opt_params, 
                                       const VehParams& veh_params,
                                       const OptimOpts& optim_opts,
                                       const QPParams& qp_params,
                                       bool debug)
    : MinCurvOptimizer(opt_params, veh_params, optim_opts, debug), qp_params_(qp_params) {
}

std::vector<double> MinCurvOptimizerQP::optimizeMinCurvature() {
    // This is a placeholder implementation
    // To use OSQP, you would need to:
    // 1. Install OSQP library
    // 2. Link against it in CMakeLists.txt
    // 3. Implement the actual OSQP interface
    
    if (debug_) {
        std::cout << "INFO: OSQP-based optimization not implemented yet" << std::endl;
        std::cout << "INFO: Falling back to gradient descent method" << std::endl;
    }
    
    // Fall back to parent class implementation
    return MinCurvOptimizer::optimizeMinCurvature();
}

MinCurvOptimizerQP::QPProblem MinCurvOptimizerQP::formulateQPProblem(
    const std::vector<Point2D>& centerline,
    const std::vector<TrackWidth>& track_widths,
    const std::vector<Point2D>& normal_vectors) const {
    
    QPProblem problem;
    
    // This would implement the full OSQP problem formulation
    // For now, it's a placeholder
    
    return problem;
}

void MinCurvOptimizerQP::buildCurvatureMatrix(
    const std::vector<Point2D>& centerline,
    const std::vector<Point2D>& normal_vectors,
    std::vector<double>& P_data,
    std::vector<int>& P_indices,
    std::vector<int>& P_indptr) const {
    
    // Placeholder for OSQP sparse matrix construction
}

void MinCurvOptimizerQP::buildConstraintMatrix(
    const std::vector<TrackWidth>& track_widths,
    std::vector<double>& A_data,
    std::vector<int>& A_indices,
    std::vector<int>& A_indptr,
    std::vector<double>& l,
    std::vector<double>& u) const {
    
    // Placeholder for OSQP constraint matrix construction
}

std::vector<double> MinCurvOptimizerQP::solveQP(const QPProblem& problem) const {
    // Placeholder for OSQP solver interface
    return std::vector<double>();
}

void MinCurvOptimizerQP::addToSparseMatrix(
    std::vector<double>& data,
    std::vector<int>& indices,
    std::vector<int>& indptr,
    int row, int col, double value) const {
    
    // Placeholder for sparse matrix utilities
}

} // namespace mincurv
