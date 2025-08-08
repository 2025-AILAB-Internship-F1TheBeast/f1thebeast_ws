import numpy as np
import time
import json
import os
import configparser
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
# FUNCTIONS ----------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

def load_track_data(file_path):
    """
    csv 파일
    [x_m, y_m, w_tr_right_m, w_tr_left_m]
    """
    try:
        track_data = np.loadtxt(file_path, delimiter=',', skiprows=1)
        print(f"INFO: Loaded track with {track_data.shape[0]} points")
        return track_data
    except Exception as e:
        print(f"ERROR: Failed to load track data: {e}")
        return None

def calculate_normal_vectors(track_points):
    """
    centerline [x, y]-> normal vectors [nx, ny]

    Input: track_points [N, 2]
    Output: normal_vectors [N, 2]
    """
    N = len(track_points)
    tangent_vectors = np.zeros((N, 2))
    
    # Calculate tangent vectors using finite differences
    for i in range(N):
        if i == 0:
            # Forward difference for first point
            tangent_vectors[i] = track_points[i+1] - track_points[i]
        elif i == N-1:
            # Backward difference for last point (connect to first)
            tangent_vectors[i] = track_points[0] - track_points[i-1]
        else:
            # Central difference for middle points
            tangent_vectors[i] = track_points[i+1] - track_points[i-1]
    
    # Normalize tangent vectors
    tangent_lengths = np.linalg.norm(tangent_vectors, axis=1)
    tangent_vectors = tangent_vectors / tangent_lengths[:, np.newaxis]
    
    # Calculate normal vectors (rotate tangent by 90 degrees)
    normal_vectors = np.zeros_like(tangent_vectors)
    normal_vectors[:, 0] = -tangent_vectors[:, 1]  # -dy
    normal_vectors[:, 1] = tangent_vectors[:, 0]   # dx
    
    return normal_vectors

def calculate_curvature_finite_diff(points):
    """
    path coordinates [x, y] -> curvature at each point

    Input: points [N, 2]
    Output: curvature [N,]
    """
    N = len(points)
    curvature = np.zeros(N)
    
    for i in range(N):
        # Get three consecutive points
        p1 = points[(i-1) % N]
        p2 = points[i]
        p3 = points[(i+1) % N]
        
        # 각 부분 1차 미분
        dx1 = p2[0] - p1[0]
        dy1 = p2[1] - p1[1]
        dx2 = p3[0] - p2[0]
        dy2 = p3[1] - p2[1]
        
        # 2차 미분
        d2x = dx2 - dx1
        d2y = dy2 - dy1
        
        # Curvature formula: κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
        denominator = (dx1**2 + dy1**2)**(3/2)
        if denominator > 1e-10:  # 분모가 0이면 안되니깐
            curvature[i] = abs(dx1 * d2y - dy1 * d2x) / denominator
        else:
            curvature[i] = 0.0
    
    return curvature

def simple_min_curvature_optimization(centerline, track_widths, normal_vectors, max_dev, max_curvature):
    """
    Inputs:
    - centerline [N, 2]: track centerline coordinates
    - track_widths [N, 2]: [right_width, left_width] 
    - normal_vectors [N, 2]: normal vectors at each point
    - max_dev: maximum allowed deviation
    - max_curvature: maximum allowed curvature
    
    Output:
    - alpha [N,]: lateral deviations from centerline
    """
    N = len(centerline)
    alpha = np.zeros(N)  # lateral deviations
    
    # Calculate track constraints
    max_right = np.minimum(track_widths[:, 0] - 0.2, max_dev)  # 0.2m safety margin
    max_left = np.minimum(track_widths[:, 1] - 0.2, max_dev)
    
    # Iterative optimization
    for iteration in range(pars["opt_params"]["max_iterations"]): # 지금은 200
        alpha_old = alpha.copy()
        
        # Calculate current path
        current_path = centerline + alpha[:, np.newaxis] * normal_vectors
        
        # Calculate current curvature
        current_curvature = calculate_curvature_finite_diff(current_path)
        
        # Objective: minimize sum of squared curvatures
        # Simple gradient descent approach
        learning_rate = pars["gd_params"]["learning_rate"] # 지금은 0.1
        
        for i in range(N):
            # Calculate gradient of curvature w.r.t. alpha[i]
            # Simple finite difference approximation
            eps = 0.01
            
            # Test positive perturbation
            alpha_test = alpha.copy()
            alpha_test[i] += eps
            test_path = centerline + alpha_test[:, np.newaxis] * normal_vectors
            curvature_plus = calculate_curvature_finite_diff(test_path)
            
            # Test negative perturbation  
            alpha_test = alpha.copy()
            alpha_test[i] -= eps
            test_path = centerline + alpha_test[:, np.newaxis] * normal_vectors
            curvature_minus = calculate_curvature_finite_diff(test_path)
            
            # Approximate gradient
            grad = np.sum(curvature_plus**2) - np.sum(curvature_minus**2)
            grad /= (2 * eps)
            
            # Update alpha
            alpha[i] -= learning_rate * grad
            
            # Apply constraints
            alpha[i] = np.clip(alpha[i], -max_left[i], max_right[i])
            
            # Apply curvature constraint (simple penalty)
            if current_curvature[i] > max_curvature:
                alpha[i] *= 0.9  # Reduce deviation to lower curvature
        
        # Check convergence
        change = np.linalg.norm(alpha - alpha_old)
        if change < pars["opt_params"]["convergence_threshold"]:
            print(f"INFO: Converged after {iteration+1} iterations")
            break
        
        if iteration % 20 == 0 and debug:
            max_curv = np.max(current_curvature)
            avg_curv = np.mean(current_curvature)
            print(f"Iteration {iteration}: max_curvature={max_curv:.4f}, avg_curvature={avg_curv:.4f}")
    
    return alpha
# resample track to desired stepsize (simple linear interpolation)
def resample_track(points, widths, target_stepsize):
    """Resample track to approximately uniform spacing"""
    # Calculate cumulative distances
    distances = np.zeros(len(points))
    for i in range(1, len(points)):
        distances[i] = distances[i-1] + np.linalg.norm(points[i] - points[i-1])
    
    total_length = distances[-1]
    n_points = int(total_length / target_stepsize)
    
    # Create new distance array
    new_distances = np.linspace(0, total_length, n_points)
    
    # Interpolate points and widths
    new_points = np.zeros((n_points, 2))
    new_widths = np.zeros((n_points, 2))
    
    new_points[:, 0] = np.interp(new_distances, distances, points[:, 0])
    new_points[:, 1] = np.interp(new_distances, distances, points[:, 1])
    new_widths[:, 0] = np.interp(new_distances, distances, widths[:, 0])
    new_widths[:, 1] = np.interp(new_distances, distances, widths[:, 1])
    
    return new_points, new_widths, new_distances
# ----------------------------------------------------------------------------------------------------------------------
# MAIN PROCESSING ------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
########################## 파라미터 파일 ##########################
file_paths = {"veh_params_file": "onlymincurv.ini"}

# select track file (including centerline coordinates + track widths) --------------------------------------------------
file_paths["track_name"] = "map_1753950751"                                    # Berlin Formula E 2018

# debug options ------------------------------------------------------------------------------------------------
debug = True                                    # print console messages
plot_path = True                                # plot global path on track map

# get current path
file_paths["module"] = os.path.dirname(os.path.abspath(__file__))

# assemble file paths
file_paths["track_file"] = os.path.join(file_paths["module"], "inputs", "tracks", file_paths["track_name"] + ".csv")
file_paths["globalpath_export"] = os.path.join(file_paths["module"], "outputs", "global_path_simple.csv")

# create outputs folder
os.makedirs(file_paths["module"] + "/outputs", exist_ok=True)

# load parameters
parser = configparser.ConfigParser()
if not parser.read(os.path.join(file_paths["module"], "params", file_paths["veh_params_file"])):
    raise ValueError('Specified config file does not exist or is empty!')

########################## 파라미터 설정 ##########################
pars = {}
pars["opt_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'opt_params'))  
# stepsize=3.0, max_iterations=100, convergence_threshold=1e-6, alpha_max=5.0
pars["veh_params"] = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params')) 
#width= 2.0, curvlim = 0.12
pars["optim_opts"] = json.loads(parser.get('OPTIMIZATION_OPTIONS', 'optim_opts_mincurv'))
#width_opt= 3.4, iqp_iters_min= 3, iqp_curverror_allowed= 0.01


########################## 경로 최적화 ##########################
# start timing
t_start = time.perf_counter()

# load track data
print("INFO: Loading track data...")
track_data = load_track_data(file_paths["track_file"])
if track_data is None:
    exit(1)

# extract centerline and track widths
centerline = track_data[:, :2]  # [x, y]
track_widths = track_data[:, 2:4]  # [w_right, w_left]

print(f"INFO: Track has {len(centerline)} points")
print(f"INFO: Track width range: {np.min(track_widths):.1f} - {np.max(track_widths):.1f} meters")

# resample track
print("INFO: Resampling track...")
centerline_resampled, widths_resampled, distances = resample_track(
    centerline, track_widths, pars["opt_params"]["stepsize"])

print(f"INFO: Resampled to {len(centerline_resampled)} points")

# calculate normal vectors
print("INFO: Calculating normal vectors...")
normal_vectors = calculate_normal_vectors(centerline_resampled)

# perform optimization
print("INFO: Starting minimum curvature optimization...")
alpha_optimal = simple_min_curvature_optimization(
    centerline_resampled,
    widths_resampled,
    normal_vectors,
    pars["optim_opts"]["width_opt"] / 2,  # max deviation
    pars["veh_params"]["curvlim"]         # max curvature
)

# calculate optimal path
optimal_path = centerline_resampled + alpha_optimal[:, np.newaxis] * normal_vectors

# calculate final curvature for 디버깅 , 학습 진행
final_curvature = calculate_curvature_finite_diff(optimal_path)

print(f"INFO: Optimization completed in {time.perf_counter() - t_start:.2f}s")
print(f"INFO: Maximum curvature: {np.max(final_curvature):.4f} rad/m")
print(f"INFO: Average curvature: {np.mean(final_curvature):.4f} rad/m")
print(f"INFO: Maximum lateral deviation: {np.max(np.abs(alpha_optimal)):.2f} m")

# prepare output data
global_path_simple = np.column_stack((distances, optimal_path))

# export to CSV
np.savetxt(file_paths["globalpath_export"], 
           global_path_simple, 
           delimiter=',', 
           fmt='%.6f',
           header='s_m,x_m,y_m',
           comments='')

print("INFO: Global path exported to:", file_paths["globalpath_export"])

########################## 결과 디버깅 ##########################
if debug:
    print("\n" + "="*60)
    print("SIMPLE GLOBAL PATH SUMMARY")
    print("="*60)
    print(f"Track: {file_paths['track_name']}")
    print(f"Optimization method: Simple finite differences")
    print(f"Number of path points: {len(optimal_path)}")
    print(f"Total path length: {distances[-1]:.2f} meters")
    print(f"Average point spacing: {pars['opt_params']['stepsize']:.1f} meters")
    print(f"Maximum curvature: {np.max(final_curvature):.4f} rad/m")
    print(f"Curvature limit: {pars['veh_params']['curvlim']:.4f} rad/m")
    print("="*60)

########################## visualization ##########################
if plot_path:
    print("\nVisualizing simple global path...")
    
    plt.figure(figsize=(12, 10))
    
    # Calculate track boundaries (simple approach)
    bound_right = centerline_resampled + widths_resampled[:, 0:1] * normal_vectors
    bound_left = centerline_resampled - widths_resampled[:, 1:2] * normal_vectors
    
    # Plot track boundaries
    plt.plot(bound_right[:, 0], bound_right[:, 1], 'k-', linewidth=2, label='Track boundary')
    plt.plot(bound_left[:, 0], bound_left[:, 1], 'k-', linewidth=2)
    # # 이까지 진행된 plot을 grid map(png)으로 저장
    # plt.savefig("grid_map.png", dpi=300, bbox_inches='tight')
    # plt.close()

    # Plot centerline
    plt.plot(centerline_resampled[:, 0], centerline_resampled[:, 1], 'b--', 
             linewidth=1, alpha=0.7, label='Centerline')
    
    # Plot optimal path
    plt.plot(optimal_path[:, 0], optimal_path[:, 1], 'r-', linewidth=3, label='Optimal Racing Line')
    
    # Mark start point
    plt.plot(optimal_path[0, 0], optimal_path[0, 1], 'go', markersize=10, label='Start/Finish')
    
    # Add distance markers
    n_markers = 10
    marker_indices = np.linspace(0, len(optimal_path)-1, n_markers, dtype=int)
    for i, idx in enumerate(marker_indices[1:-1]):
        plt.plot(optimal_path[idx, 0], optimal_path[idx, 1], 'yo', markersize=6, alpha=0.8)
        plt.text(optimal_path[idx, 0], optimal_path[idx, 1], f'{distances[idx]:.0f}m', 
                fontsize=8, ha='center', va='bottom', alpha=0.8)
    
    # Set plot properties
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title(f'Simple Minimum Curvature Racing Line - {file_paths["track_name"]}')
    plt.legend()
    
    # Add info box
    textstr = f'Track: {file_paths["track_name"]}\nPath Length: {distances[-1]:.1f} m\nMax Curvature: {np.max(final_curvature):.3f} rad/m'
    props = dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
    plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', bbox=props)
    
    plt.tight_layout()
    
    # Save plot
    plot_save_path = os.path.join(file_paths["module"], "outputs", f"simple_path_{file_paths['track_name']}.png")
    plt.savefig(plot_save_path, dpi=300, bbox_inches='tight')
    print(f"INFO: Plot saved to: {plot_save_path}")
    
    plt.show()

print("INFO: Simple minimum curvature optimization completed!")
