#!/usr/bin/env python3
"""
Map Converter for F1Tenth Racing with Advanced Preprocessing
Converts a .png or .pgm + .yaml map into a .csv file that can be fed into TUMFTM functions.

HIGH LEVEL STEPS:
1. Load map image and metadata
2. Apply SLAM-based pruned branches preprocessing to clean track structure
3. Apply skeletonization to the preprocessed map to extract centerlines
4. Run DFS to extract the centerline xy coordinates in order
5. Apply transformations to go from pixel to meter coordinate frame

INPUT REQUIREMENTS:
- Map image file: .png or .pgm format in 'maps/' directory
- Map metadata file: .yaml with same basename containing:
  * resolution: meters per pixel (float)
  * origin: [x, y, theta] offset in meters (list)
- Map conditions:
  * Map edges should be well defined
  * Inside track should be white (high pixel values)
  * Walls should be black (low pixel values)
  * Outside areas should be grey (medium pixel values)

PREPROCESSING FEATURES:
- SLAM-based branch pruning algorithm removes unwanted spurs and branches
- Erosion-based track cleaning for better centerline extraction
- NetworkX graph analysis to preserve main track loops while removing branches

OUTPUT FORMAT:
- CSV file saved to 'inputs/tracks/' directory
- Columns: x_m, y_m, w_tr_right_m, w_tr_left_m
- x_m, y_m: centerline coordinates in meters (relative to map origin)
- w_tr_right_m, w_tr_left_m: track width on right/left side in meters

PARAMETERS:
- BINARY_THRESHOLD: 0-255 (grayscale threshold)
- DISTANCE_THRESHOLD: 0.0-1.0 (fraction of max distance)
- APPLY_PRUNED_PREPROCESSING: True/False (enable advanced preprocessing)
- EROSION_ITERATIONS: 1-3 (morphological erosion iterations)
- LEFT_START_Y_OFFSET: Should be within map bounds when added to map_height//2

POTENTIAL ISSUES:
- IndexError: Check LEFT_START_Y_OFFSET and map dimensions
- No centerline found: Adjust DISTANCE_THRESHOLD or disable preprocessing
- Empty output: Verify map has clear track structure
- NetworkX import error: Ensure networkx is installed (pip install networkx)
"""
#morphologyEX
import numpy as np
from skimage.morphology import skeletonize
import matplotlib
matplotlib.use('Agg')  # GUI 없이 실행하기 위한 백엔드 설정
import matplotlib.pyplot as plt
import yaml
import scipy.ndimage
from PIL import Image
import os
import sys
import cv2
import networkx as nx
from skimage.morphology import remove_small_holes, binary_closing, disk
# =============================================================================
# PARAMETERS - Configure these values for your specific map
# =============================================================================

# Map configuration
MAP_NAME = "map_1753950751"
#MAP_NAME = "first_map"                # Name of the map file (without extension)
TRACK_WIDTH_MARGIN = 0.0                 # Extra safety margin, in meters

# Image processing parameters
BINARY_THRESHOLD = 210.0                 # Threshold for converting grayscale to binary (0-255)
DISTANCE_THRESHOLD = 0.17                # Threshold for distance transform (0.0-1.0)
                                        # If you see hairy lines, increase this number
APPLY_PRUNED_PREPROCESSING = True        # Apply SLAM-based pruned branches preprocessing
EROSION_ITERATIONS = 2                   # Number of erosion iterations for preprocessing

# DFS parameters
LEFT_START_Y_OFFSET = -120              # Offset from map center for finding start point
RECURSION_LIMIT = 20000                 # Maximum recursion depth for DFS

# Output configuration
OUTPUT_DIR = "inputs/tracks"            # Directory to save the CSV file
OUTPUT_FORMAT = '%0.4f'                # Format for saving numbers in CSV
DOWNSAMPLE_FACTOR = None               # Set to integer (e.g., 4) to downsample data, None to keep all

# Visualization parameters
FIGURE_SIZE = (10, 10)                 # Size of matplotlib figures

# =============================================================================
# PRUNED BRANCHES ALGORITHM FROM SLAM MAP PROCESSING
# =============================================================================

def skeleton_to_graph(skeleton):
    """Convert skeleton image to NetworkX graph for branch analysis."""
    G = nx.Graph()
    h, w = skeleton.shape
    for y in range(h):
        for x in range(w):
            if skeleton[y, x]:
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx_ = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx_ < w and skeleton[ny, nx_]:
                            G.add_edge((y, x), (ny, nx_))
    return G

def apply_pruned_branches_preprocessing(map_img, erosion_iterations=1):
    """
    Apply pruned branches algorithm to clean up the track before centerline extraction.
    
    Args:
        map_img: Binary map image (1=free space, 0=occupied)
        erosion_iterations: Number of erosion iterations to apply
    
    Returns:
        cleaned_map: Preprocessed map with unwanted branches removed
    """
    print(f"   Applying pruned branches preprocessing...")
    print(f"   Original free space pixels: {np.sum(map_img == 1)}")

    #------------------------------------------------------------------
    map_img_bool = map_img.astype(bool)

    map_img_bool = remove_small_holes(
        map_img_bool,
        area_threshold=100,   # 픽셀 수: 잡음보다 조금 크게 잡기
        connectivity=2        # 8-연결
    )
    binary_map = (map_img_bool.astype(np.uint8) * 255)
    #------------------------------------------------------------------

    # Convert to uint8 format for morphological operations
    #binary_map = (map_img * 255).astype(np.uint8)
    
    # Apply erosion to thin out the track
    if erosion_iterations > 0:
        #-------------erosion kernel-------------- 3,3   5,5
        kernel = np.ones((5, 5), np.uint8)
        eroded = cv2.erode(binary_map, kernel, iterations=erosion_iterations)
        #현재 픽셀 값 0 또는 255
        print(f"   After erosion: {np.sum(eroded > 0)} pixels")
    else:
        eroded = binary_map
    
    # Apply skeletonization
    skeleton = skeletonize(eroded // 255).astype(np.uint8)
    print(f"   After skeletonization: {np.sum(skeleton > 0)} pixels")
    #0 또는 1
    # Convert skeleton to graph for branch analysis
    G = skeleton_to_graph(skeleton)
    print(f"   Graph nodes: {len(G.nodes)}, edges: {len(G.edges)}")
    
    # Find loops and endpoints for branch pruning
    loops = list(nx.cycle_basis(G))
    loop_nodes = set([n for cycle in loops for n in cycle])
    endpoints = [n for n in G.nodes if G.degree[n] == 1]
    #인접 노드 수 체크로 엔드 포인트 남김 
    
    print(f"   Found {len(loops)} loops with {len(loop_nodes)} loop nodes")
    print(f"   Found {len(endpoints)} endpoints")
    
    # Mark nodes for removal
    to_remove = set()
    visited = set()
    
    for ep in endpoints:
        if ep in visited:
            continue
        
        stack = [(ep, [ep])]
        while stack:
            current, trace = stack.pop()
            visited.add(current)
            neighbors = [n for n in G.neighbors(current) if n not in visited]
            
            # If any neighbor is in loop_nodes, keep this branch
            if any(n in loop_nodes for n in neighbors):
                continue  # Keep branches connected to loops
            else:
                # Remove branches not connected to loops
                to_remove.update(trace)
            
            for n in neighbors:
                stack.append((n, trace + [n]))
    
    # Apply removal to skeleton
    cleaned_skeleton = skeleton.copy()
    for y, x in to_remove:
        cleaned_skeleton[y, x] = 0
    
    print(f"   After branch pruning: {np.sum(cleaned_skeleton > 0)} pixels")
    print(f"   Removed {len(to_remove)} branch pixels")
    
    # Dilate the cleaned skeleton back to create a cleaned track
    if erosion_iterations > 0:
        # Dilate to restore some width
        #-------------dilation kernel--------------
        kernel = np.ones((3, 3), np.uint8)
        cleaned_thick = cv2.dilate(cleaned_skeleton, kernel, iterations=erosion_iterations)
    else:
        cleaned_thick = cleaned_skeleton
    
    # Convert back to binary format (0-1)
    cleaned_map = (cleaned_thick > 0).astype(np.float64)
    
    print(f"   Final cleaned map: {np.sum(cleaned_map == 1)} pixels")
    
    return cleaned_map

# =============================================================================
# MAIN CONVERSION FUNCTIONS
# =============================================================================

def load_map(map_name):
    """Load map image and yaml metadata."""
    # Find map file
    if os.path.exists(f"maps/{map_name}.png"):
        map_img_path = f"maps/{map_name}.png"
    elif os.path.exists(f"maps/{map_name}.pgm"):
        map_img_path = f"maps/{map_name}.pgm"
    else:
        raise Exception(f"Map not found! Looking for maps/{map_name}.png or maps/{map_name}.pgm")
    
    map_yaml_path = f"maps/{map_name}.yaml"
    
    # Load image
    raw_map_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
    raw_map_img = raw_map_img.astype(np.float64)
    
    # Load yaml metadata
    with open(map_yaml_path, 'r') as yaml_stream:
        try:
            map_metadata = yaml.safe_load(yaml_stream)
            map_resolution = map_metadata['resolution']
            origin = map_metadata['origin']
        except yaml.YAMLError as ex:
            print(f"Error loading YAML: {ex}")
            raise
    
    return raw_map_img, map_resolution, origin

def convert_to_binary(raw_map_img, threshold=BINARY_THRESHOLD, apply_pruned_preprocessing=True):
    """Convert grayscale image to binary and optionally apply pruned branches preprocessing."""
    map_img = raw_map_img.copy()
    map_img[map_img <= threshold] = 0
    map_img[map_img > threshold] = 1
    
    if apply_pruned_preprocessing:
        print("   Applying pruned branches preprocessing to clean up track structure...")
        # Apply the SLAM-based pruned branches algorithm
        map_img = apply_pruned_branches_preprocessing(map_img, erosion_iterations=EROSION_ITERATIONS)
    
    return map_img

def extract_centerline(map_img, distance_threshold=DISTANCE_THRESHOLD):
    """Extract centerline using distance transform and skeletonization."""
    # Calculate Euclidean Distance Transform
    dist_transform = scipy.ndimage.distance_transform_edt(map_img)
    
    # Threshold the distance transform
    centers = dist_transform > distance_threshold * dist_transform.max()
    
    # Apply skeletonization
    centerline = skeletonize(centers)
    
    # Encode track width information
    centerline_dist = np.where(centerline, dist_transform, 0)
    
    return centerline_dist, dist_transform

def find_start_point(centerline_dist, map_height, y_offset=LEFT_START_Y_OFFSET):
    """Find starting point for DFS traversal."""
    NON_EDGE = 0.0
    left_start_y = map_height // 2 + y_offset
    
    # Ensure y coordinate is within bounds
    left_start_y = max(0, min(left_start_y, map_height - 1))
    left_start_x = 0
    
    # Find the first non-zero point in the row
    map_width = centerline_dist.shape[1]
    while (left_start_x < map_width and centerline_dist[left_start_y][left_start_x] == NON_EDGE):
        left_start_x += 1
    
    # If no point found in this row, search nearby rows
    if left_start_x >= map_width:
        print(f"No centerline found at y={left_start_y}, searching nearby rows...")
        found = False
        for search_range in range(1, min(100, map_height//4)):  # Search within reasonable range
            for dy in [-search_range, search_range]:
                test_y = left_start_y + dy
                if 0 <= test_y < map_height:
                    for x in range(map_width):
                        if centerline_dist[test_y][x] != NON_EDGE:
                            left_start_y = test_y
                            left_start_x = x
                            found = True
                            break
                    if found:
                        break
            if found:
                break
        
        if not found:
            raise Exception("Could not find any centerline points to start DFS. Check your distance threshold and map.")
    
    print(f"Starting position for left edge: {left_start_x} {left_start_y}")
    return left_start_x, left_start_y

def extract_centerline_points(centerline_dist, start_x, start_y, recursion_limit=RECURSION_LIMIT):
    """Extract centerline points using DFS."""
    sys.setrecursionlimit(recursion_limit)
    
    visited = {}
    centerline_points = []
    track_widths = []
    NON_EDGE = 0.0
    
    map_height, map_width = centerline_dist.shape
    
    # Direction priority: prefer certain directions first
    DIRECTIONS = [(0, -1), (-1, 0), (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]
    
    def dfs(point):
        if point in visited:
            return
        visited[point] = True
        centerline_points.append(np.array(point))
        track_widths.append(np.array([centerline_dist[point[1]][point[0]], 
                                    centerline_dist[point[1]][point[0]]]))
        
        for direction in DIRECTIONS:
            new_x = point[0] + direction[0]
            new_y = point[1] + direction[1]
            
            # Enhanced bounds checking
            if (0 <= new_x < map_width and 
                0 <= new_y < map_height and
                centerline_dist[new_y][new_x] != NON_EDGE and 
                (new_x, new_y) not in visited):
                dfs((new_x, new_y))
    
    # Validate starting point
    if not (0 <= start_x < map_width and 0 <= start_y < map_height):
        raise Exception(f"Starting point ({start_x}, {start_y}) is out of bounds for map size ({map_width}, {map_height})")
    
    starting_point = (start_x, start_y)
    dfs(starting_point)
    
    if len(centerline_points) == 0:
        raise Exception("No centerline points extracted. Check your parameters and map.")
    
    return centerline_points, track_widths

def convert_to_meters(centerline_points, track_widths, map_resolution, origin, 
                     track_width_margin=TRACK_WIDTH_MARGIN):
    """Convert pixel coordinates to meter coordinates."""
    # Convert to numpy arrays
    track_widths_np = np.array(track_widths)
    waypoints = np.array(centerline_points)
    
    print(f"Track widths shape: {track_widths_np.shape}, waypoints shape: {waypoints.shape}")
    
    # Merge track with waypoints
    data = np.concatenate((waypoints, track_widths_np), axis=1)
    
    # Calculate map parameters
    orig_x = origin[0]
    orig_y = origin[1]
    
    # Apply transformations
    transformed_data = data.copy()
    transformed_data *= map_resolution
    transformed_data += np.array([orig_x, orig_y, 0, 0])
    
    # Apply safety margin
    transformed_data -= np.array([0, 0, track_width_margin, track_width_margin])
    
    return transformed_data

def save_csv(data, map_name, output_dir=OUTPUT_DIR, output_format=OUTPUT_FORMAT, 
             downsample_factor=DOWNSAMPLE_FACTOR):
    """Save transformed data to CSV file."""
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Apply downsampling if specified
    if downsample_factor is not None:
        data = data[::downsample_factor]
        print(f"Downsampled data by factor of {downsample_factor}")
    
    # Save to CSV
    output_path = os.path.join(output_dir, f"{map_name}.csv")
    with open(output_path, 'wb') as fh:
        np.savetxt(fh, data, fmt=output_format, delimiter=',', 
                  header='x_m,y_m,w_tr_right_m,w_tr_left_m')
    
    print(f"Saved converted track data to: {output_path}")
    return output_path

def show_preprocessing_comparison(raw_map_img, map_img_original, map_img_preprocessed, 
                                figure_size=FIGURE_SIZE):
    """Show comparison between original and preprocessed binary maps."""
    fig, axes = plt.subplots(1, 3, figsize=(figure_size[0]*3, figure_size[1]))
    
    axes[0].imshow(raw_map_img, cmap='gray', origin='lower')
    axes[0].set_title("Original Grayscale Map")
    axes[0].axis('off')
    
    axes[1].imshow(map_img_original, cmap='gray', origin='lower')
    axes[1].set_title("Binary Conversion with pruned branches")
    axes[1].axis('off')
    
    axes[2].imshow(map_img_preprocessed, cmap='gray', origin='lower')
    axes[2].set_title("Centerline")
    axes[2].axis('off')
    
    plt.tight_layout()
    plt.savefig('preprocessing_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("   Preprocessing comparison saved: preprocessing_comparison.png")

# =============================================================================
# MAIN EXECUTION
# =============================================================================

def convert_map_to_csv(map_name=MAP_NAME, show_plots=True):
    """
    Main function to convert map to CSV.
    
    Args:
        map_name (str): Name of the map to convert
        show_plots (bool): Whether to show intermediate processing plots
    
    Returns:
        str: Path to the generated CSV file
    
    Input Requirements:
        - Map image: .png or .pgm format in 'maps/' directory
        - Map metadata: .yaml file with resolution and origin
        - Map should have clear white tracks, black walls, grey outside areas
    
    Output Format:
        - CSV file with columns: x_m, y_m, w_tr_right_m, w_tr_left_m
        - Coordinates in meters relative to map origin
        - Track widths based on distance transform
    """
    print(f"Converting map: {map_name}")
    print("="*50)
    
    # Step 1: Load map
    print("1. Loading map...")
    raw_map_img, map_resolution, origin = load_map(map_name)
    print(f"   Map resolution: {map_resolution} m/pixel")
    print(f"   Map origin: {origin}")
    print(f"   Map shape: {raw_map_img.shape}")

    # Step 2: Convert to binary and apply preprocessing
    print("2. Converting to binary and applying preprocessing...")
    map_img = convert_to_binary(raw_map_img, apply_pruned_preprocessing=APPLY_PRUNED_PREPROCESSING)
    map_height, map_width = map_img.shape
    print(f"   Final processed map shape: {map_height} x {map_width}")
    print(f"   Free space pixels: {np.sum(map_img == 1)}")
    print(f"   Occupied pixels: {np.sum(map_img == 0)}")
    
    # Step 3: Extract centerline
    print("3. Extracting centerline...")
    centerline_dist, dist_transform = extract_centerline(map_img)
    print(f"   Max distance from walls: {dist_transform.max():.2f} pixels")
    centerline_pixels = np.sum(centerline_dist > 0)
    print(f"   Centerline pixels found: {centerline_pixels}")
    
    if centerline_pixels == 0:
        raise Exception(f"No centerline found! Try adjusting DISTANCE_THRESHOLD (current: {DISTANCE_THRESHOLD})")
    
    # Step 4: Find start point and extract points
    print("4. Finding start point and extracting centerline points...")
    print(f"   Using LEFT_START_Y_OFFSET: {LEFT_START_Y_OFFSET}")
    print(f"   Calculated start Y: {map_height // 2 + LEFT_START_Y_OFFSET}")
    start_x, start_y = find_start_point(centerline_dist, map_height)
    centerline_points, track_widths = extract_centerline_points(centerline_dist, start_x, start_y)
    print(f"   Extracted {len(centerline_points)} centerline points")
    
    # Step 5: Convert to meters
    print("5. Converting to meter coordinates...")
    transformed_data = convert_to_meters(centerline_points, track_widths, map_resolution, origin)
    
    # Step 6: Save CSV
    print("6. Saving to CSV...")
    output_path = save_csv(transformed_data, map_name)
    
    # Show visualizations if requested
    if show_plots:
        print("7. Showing visualizations...")
        show_preprocessing_comparison(raw_map_img, map_img, centerline_dist)

    print("="*50)
    print(f"Conversion completed successfully!")
    print(f"Output file: {output_path}")
    print(f"Total points: {len(centerline_points)}")
    print(f"Output format: x_m, y_m, w_tr_right_m, w_tr_left_m")
    print("\nNext: Check 'sanity_check.ipynb' to verify the centerline aligns with the map.")
    
    return output_path

if __name__ == "__main__":
    # You can modify these parameters or pass command line arguments
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert F1Tenth map to CSV format')
    parser.add_argument('--map_name', '-m', default=MAP_NAME, 
                       help=f'Name of the map to convert (default: {MAP_NAME})')
    parser.add_argument('--no_plots', action='store_true', 
                       help='Skip showing intermediate plots')
    parser.add_argument('--no_preprocessing', action='store_true', 
                       help='Disable pruned branches preprocessing')
    parser.add_argument('--erosion_iterations', type=int, default=EROSION_ITERATIONS,
                       help=f'Number of erosion iterations for preprocessing (default: {EROSION_ITERATIONS})')
    parser.add_argument('--list_maps', action='store_true', 
                       help='List available maps and exit')
    
    args = parser.parse_args()
    
    # List available maps if requested
    if args.list_maps:
        print("Available maps in 'maps/' directory:")
        print("="*40)
        if os.path.exists('maps'):
            map_files = []
            for file in os.listdir('maps'):
                if file.endswith('.png') or file.endswith('.pgm'):
                    base_name = file.rsplit('.', 1)[0]
                    yaml_file = f'maps/{base_name}.yaml'
                    if os.path.exists(yaml_file):
                        map_files.append(base_name)
                        print(f"✓ {base_name}")
                    else:
                        print(f"✗ {base_name} (missing .yaml file)")
            print(f"\nTotal valid maps: {len(map_files)}")
        else:
            print("'maps/' directory not found!")
        sys.exit(0)
    
    # Update global parameters if provided
    if args.map_name != MAP_NAME:
        MAP_NAME = args.map_name
    
    if args.no_preprocessing:
        APPLY_PRUNED_PREPROCESSING = False
        print("Pruned branches preprocessing disabled.")
    
    if args.erosion_iterations != EROSION_ITERATIONS:
        EROSION_ITERATIONS = args.erosion_iterations
        print(f"Using {EROSION_ITERATIONS} erosion iterations.")
    
    try:
        convert_map_to_csv(
            map_name=args.map_name,
            show_plots=not args.no_plots
        )
    except Exception as e:
        print(f"Error during conversion: {e}")
        print(f"\nTroubleshooting tips:")
        print(f"1. Check if maps/{args.map_name}.png/.pgm and maps/{args.map_name}.yaml exist")
        print(f"2. Verify map has clear track structure (white=free, black=walls)")
        print(f"3. Try adjusting DISTANCE_THRESHOLD (current: {DISTANCE_THRESHOLD})")
        print(f"4. Try adjusting LEFT_START_Y_OFFSET (current: {LEFT_START_Y_OFFSET})")
        print(f"5. Use --list_maps to see available maps")
        sys.exit(1)
