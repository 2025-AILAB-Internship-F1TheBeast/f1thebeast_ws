import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import yaml
import os
import csv
from skimage.morphology import skeletonize

# 이미지와 YAML 경로 설정
map_img_path = "/home/yongwoo/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map.png"
map_yaml_path = "/home/yongwoo/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map.yaml"

def extract_spielberg_track():
    if not os.path.exists(map_img_path):
        raise Exception(f"Map image not found: {map_img_path}")
    if not os.path.exists(map_yaml_path):
        print(f"Warning: YAML not found: {map_yaml_path}")
        yaml_data = None
    else:
        with open(map_yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        print(f"YAML loaded: resolution={yaml_data.get('resolution', 'unknown')}")

    raw_img = np.array(Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM))
    raw_img = raw_img.astype(np.uint8)

    print(f"Image shape: {raw_img.shape}, min/max: {raw_img.min()}/{raw_img.max()}")

    # 이진화
    threshold = 200
    _, binary = cv2.threshold(raw_img, threshold, 255, cv2.THRESH_BINARY)
    print(f"Binary threshold applied. White: {np.sum(binary == 255)}, Black: {np.sum(binary == 0)}")

    def method_fill_inner_by_center(binary_img):
        h, w = binary_img.shape
        result = np.zeros_like(binary_img)
        flood_filled = binary_img.copy()
        cy, cx = h // 2, w // 2

        # 중심에서 시작해 트랙 내부를 Flood Fill
        if flood_filled[cy, cx] == 0:
            found = False
            for radius in range(1, max(h, w)):
                for dy in range(-radius, radius+1):
                    for dx in range(-radius, radius+1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and flood_filled[ny, nx] == 255:
                            cy, cx = ny, nx
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
        mask = np.zeros((h+2, w+2), np.uint8)
        cv2.floodFill(flood_filled, mask, (cx, cy), 128)
        result[flood_filled == 128] = 255
        return result

    result_center = method_fill_inner_by_center(binary)
    return raw_img, binary, result_center, yaml_data

def extract_centerline_and_overlay(binary_mask):
    # 거리 변환 및 중심선 추출
    dist_transform = cv2.distanceTransform(binary_mask, distanceType=cv2.DIST_L2, maskSize=5)
    THRESHOLD = 0.17
    centers = dist_transform > (THRESHOLD * dist_transform.max())
    centerline = skeletonize(centers)
    centerline_dist = np.where(centerline, dist_transform, 0)

    map_height, map_width = centerline.shape
    LEFT_START_Y = map_height // 2 - 120
    NON_EDGE = 0.0

    # 중심선 시작점 탐색
    left_start_y = LEFT_START_Y
    left_start_x = 0
    while left_start_x < map_width and centerline_dist[left_start_y][left_start_x] == NON_EDGE:
        left_start_x += 1

    print(f"Starting position for DFS: ({left_start_x}, {left_start_y})")

    # DFS로 중심선 연결
    import sys
    sys.setrecursionlimit(20000)
    visited = {}
    centerline_points = []
    track_widths = []
    DIRECTIONS = [(0, -1), (-1, 0),  (0, 1), (1, 0), (-1, 1), (-1, -1), (1, 1), (1, -1)]

    def dfs(point):
        if point in visited:
            return
        visited[point] = True
        centerline_points.append(np.array(point))
        width = centerline_dist[point[1]][point[0]]
        track_widths.append(np.array([width, width]))
        for dx, dy in DIRECTIONS:
            nx, ny = point[0] + dx, point[1] + dy
            if 0 <= nx < map_width and 0 <= ny < map_height:
                if centerline_dist[ny][nx] != NON_EDGE and (nx, ny) not in visited:
                    dfs((nx, ny))

    dfs((left_start_x, left_start_y))

    # 중심선 연두색 오버레이
    overlay = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
    for x, y in centerline_points:
        overlay[y, x] = [255, 0, 0]  # 연두색

    return centerline_points, track_widths, overlay

def visualize_all(raw_img, binary, floodfill, overlay):
    fig, axes = plt.subplots(1, 3, figsize=(22, 6))
    images = [raw_img, floodfill, overlay]
    titles = [ "original", "(track extrackted)Flood Fill", "centerline_extrackted"]
    cmaps = [ 'gray', 'gray', None]

    for ax, img, title, cmap in zip(axes, images, titles, cmaps):
        ax.imshow(img, cmap=cmap, origin='lower')
        ax.set_title(title)
        ax.axis("off")
    plt.tight_layout()
    plt.show()

def save_centerline_to_csv(centerline_points, track_widths, yaml_info, output_file="spielberg_centerline.csv"):
    """
    중심선과 폭 정보를 CSV 파일로 저장
    
    Args:
        centerline_points: 중심선 포인트들 (픽셀 좌표)
        track_widths: 각 포인트에서의 트랙 폭 정보
        yaml_info: YAML 파일에서 읽은 메타데이터 (resolution, origin 등)
        output_file: 출력할 CSV 파일명
    """
    if not centerline_points:
        print("중심선 포인트가 없습니다.")
        return
    
    # YAML 정보에서 좌표 변환 파라미터 추출
    resolution = yaml_info.get('resolution', 0.05) if yaml_info else 0.05  # 기본값 0.05m/pixel
    origin_x = yaml_info.get('origin', [0, 0, 0])[0] if yaml_info else 0
    origin_y = yaml_info.get('origin', [0, 0, 0])[1] if yaml_info else 0
    
    print(f"좌표 변환 파라미터:")
    print(f"  Resolution: {resolution} m/pixel")
    print(f"  Origin: ({origin_x}, {origin_y})")
    
    # CSV 파일 생성
    with open(output_file, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile)
        
        # 헤더 작성
        writer.writerow(['#x_m', 'y_m', 'w_tr_right_m', 'w_tr_left_m'])
        
        for i, (point, width) in enumerate(zip(centerline_points, track_widths)):
            x_pixel, y_pixel = point[0], point[1]
            width_pixel = width[0]  # 좌우 동일하다고 가정
            
            # 픽셀 좌표를 실제 좌표(미터)로 변환
            x_meter = origin_x + x_pixel * resolution
            y_meter = origin_y + y_pixel * resolution
            
            # 폭을 미터로 변환 (양쪽으로 나누어 배치)
            width_meter = width_pixel * resolution
            w_tr_right = width_meter / 2.0
            w_tr_left = width_meter / 2.0
            
            # CSV 행 작성
            writer.writerow([
                f"{x_meter:.6f}",
                f"{y_meter:.6f}", 
                f"{w_tr_right:.6f}",
                f"{w_tr_left:.6f}",
                f"{x_pixel}",
                f"{y_pixel}",
                f"{width_pixel:.6f}"
            ])
    
    print(f"중심선 데이터가 '{output_file}' 파일로 저장되었습니다.")
    print(f"총 {len(centerline_points)}개의 포인트가 저장되었습니다.")
    
    # 통계 정보 출력
    if track_widths:
        widths_pixel = [w[0] for w in track_widths]
        widths_meter = [w * resolution for w in widths_pixel]
        
        print(f"\n트랙 폭 통계:")
        print(f"  픽셀 단위 - 최소: {min(widths_pixel):.2f}, 최대: {max(widths_pixel):.2f}, 평균: {np.mean(widths_pixel):.2f}")
        print(f"  미터 단위 - 최소: {min(widths_meter):.3f}m, 최대: {max(widths_meter):.3f}m, 평균: {np.mean(widths_meter):.3f}m")

# 실행
if __name__ == "__main__":
    try:
        raw_img, binary, result_center, yaml_info = extract_spielberg_track()
        centerline_points, track_widths, overlay = extract_centerline_and_overlay(result_center)
        visualize_all(raw_img, binary, result_center, overlay)
        
        # CSV 파일로 저장
        output_csv = "/home/yongwoo/f1thebeast_ws/minimum_curvature/spielberg_centerline.csv"
        save_centerline_to_csv(centerline_points, track_widths, yaml_info, output_csv)
        
    except Exception as e:
        print(f"에러 발생: {e}")
        import traceback
        traceback.print_exc()
