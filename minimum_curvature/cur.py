import numpy as np
import cv2
from PIL import Image
import matplotlib
matplotlib.use('Agg')  # GUI 없이 실행하기 위한 백엔드 설정
import matplotlib.pyplot as plt
import yaml
import os
from skimage.morphology import skeletonize, remove_small_objects, label
from skimage.measure import regionprops
from skimage.graph import route_through_array
import networkx as nx

map_img_path = "map_1753945518.pgm"
map_yaml_path = "map_1753945518.yaml"

# ======== 맵 로드 및 전처리 ========
def extract_track():
    if not os.path.exists(map_img_path):
        raise Exception(f"Map image not found: {map_img_path}")
    if not os.path.exists(map_yaml_path):
        print(f"Warning: YAML not found: {map_yaml_path}")
        yaml_data = None
    else:
        with open(map_yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        print(f"YAML loaded: resolution={yaml_data.get('resolution', 'unknown')}")

    raw_img = np.array(Image.open(map_img_path).convert("L").transpose(Image.FLIP_TOP_BOTTOM))
    raw_img = raw_img.astype(np.uint8)
    print(f"Image shape: {raw_img.shape}, min/max: {raw_img.min()}/{raw_img.max()}")

    # 밝은 부분만 살림 (흰색 트랙)
    binary = np.where(raw_img >= 240, 255, 0).astype(np.uint8)
    print(f"After threshold: white={np.sum(binary==255)}, black={np.sum(binary==0)}")

    # Flood Fill로 닫힌 내부만 채움
    def flood_fill_inside(binary_img):
        h, w = binary_img.shape
        mask = np.zeros((h+2, w+2), np.uint8)
        flood_filled = binary_img.copy()
        cy, cx = h // 2, w // 2
        if flood_filled[cy, cx] == 0:
            for r in range(1, max(h, w)):
                for dy in range(-r, r+1):
                    for dx in range(-r, r+1):
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < h and 0 <= nx < w and flood_filled[ny, nx] == 255:
                            cy, cx = ny, nx
                            break
                    else: continue
                    break
                else: continue
                break
        cv2.floodFill(flood_filled, mask, (cx, cy), 128)
        result = np.zeros_like(binary_img)
        result[flood_filled == 128] = 255
        return result

    filled = flood_fill_inside(binary)
    return raw_img, binary, filled, yaml_data

# ======== 기본 중심선 추출 ========
def extract_centerline(filled_mask):
    dist = cv2.distanceTransform(filled_mask, cv2.DIST_L2, 5)
    binary_mask = (filled_mask > 0).astype(np.uint8)
    skeleton = skeletonize(binary_mask.astype(bool))
    centerline_mask = (skeleton.astype(np.uint8)) * 255
    return centerline_mask

# ======== 가지 제거 및 메인 루프 추출 ========
def skeleton_to_graph(skeleton):
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

def extract_pruned_centerline(filled_mask, min_branch_length=40):
    skeleton = skeletonize(filled_mask // 255).astype(np.uint8)
    G = skeleton_to_graph(skeleton)
    loops = list(nx.cycle_basis(G))
    loop_nodes = set([n for cycle in loops for n in cycle])
    endpoints = [n for n in G.nodes if G.degree[n] == 1]
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

            # 루프 접촉 가지도 제거
            if any(n in loop_nodes for n in neighbors):
                continue
            else:
                to_remove.update(trace)
            for n in neighbors:
                stack.append((n, trace + [n]))

    cleaned = skeleton.copy()
    for y, x in to_remove:
        cleaned[y, x] = 0
    return (cleaned * 255).astype(np.uint8)

def extract_main_loop_only(filled_mask):
    skeleton = skeletonize(filled_mask // 255)
    labeled = label(skeleton)
    props = regionprops(labeled)
    largest_label = max(props, key=lambda x: x.area).label
    cleaned = (labeled == largest_label).astype(np.uint8) * 255
    return cleaned

def extract_main_loop_after_erosion(filled_mask, erosion_iter=1):
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(filled_mask, kernel, iterations=erosion_iter)
    skeleton = skeletonize(eroded // 255)
    labeled = label(skeleton)
    props = regionprops(labeled)
    if not props:
        print("No region found after erosion.")
        return (skeleton.astype(np.uint8)) * 255
    largest_label = max(props, key=lambda x: x.area).label
    cleaned = (labeled == largest_label).astype(np.uint8) * 255
    return cleaned

# ======== Zhang–Suen thinning 및 메디안 중심선 추출 ========
def zhang_suen_thinning(img):
    """Zhang–Suen thinning algorithm: returns a skeleton image (0/1)."""
    skeleton = img.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        # 1단계
        to_remove = []
        h, w = skeleton.shape
        for y in range(1, h-1):
            for x in range(1, w-1):
                if skeleton[y, x] == 1:
                    p2 = skeleton[y-1, x]
                    p3 = skeleton[y-1, x+1]
                    p4 = skeleton[y,   x+1]
                    p5 = skeleton[y+1, x+1]
                    p6 = skeleton[y+1, x]
                    p7 = skeleton[y+1, x-1]
                    p8 = skeleton[y,   x-1]
                    p9 = skeleton[y-1, x-1]
                    neighbors = [p2,p3,p4,p5,p6,p7,p8,p9]
                    Np = sum(neighbors)
                    # 0→1 전이 횟수
                    transitions = 0
                    for k in range(8):
                        if neighbors[k] == 0 and neighbors[(k+1)%8] == 1:
                            transitions += 1
                    if 2 <= Np <= 6 and transitions == 1 and (p2 * p4 * p6 == 0) and (p4 * p6 * p8 == 0):
                        to_remove.append((y,x))
        if to_remove:
            changed = True
            for y, x in to_remove:
                skeleton[y, x] = 0
        # 2단계
        to_remove = []
        for y in range(1, h-1):
            for x in range(1, w-1):
                if skeleton[y, x] == 1:
                    p2 = skeleton[y-1, x]
                    p3 = skeleton[y-1, x+1]
                    p4 = skeleton[y,   x+1]
                    p5 = skeleton[y+1, x+1]
                    p6 = skeleton[y+1, x]
                    p7 = skeleton[y+1, x-1]
                    p8 = skeleton[y,   x-1]
                    p9 = skeleton[y-1, x-1]
                    neighbors = [p2,p3,p4,p5,p6,p7,p8,p9]
                    Np = sum(neighbors)
                    transitions = 0
                    for k in range(8):
                        if neighbors[k] == 0 and neighbors[(k+1)%8] == 1:
                            transitions += 1
                    if 2 <= Np <= 6 and transitions == 1 and (p2 * p4 * p8 == 0) and (p2 * p6 * p8 == 0):
                        to_remove.append((y,x))
        if to_remove:
            changed = True
            for y, x in to_remove:
                skeleton[y, x] = 0
    return skeleton

def remove_endpoints(skeleton, iterations=10):
    """Endpoint pruning: iteratively remove pixels with only one neighbor."""
    s = skeleton.copy().astype(np.uint8)
    for _ in range(iterations):
        to_remove = []
        h, w = s.shape
        for y in range(1, h-1):
            for x in range(1, w-1):
                if s[y, x]:
                    nb = [
                        s[y-1, x], s[y-1, x+1], s[y, x+1], s[y+1, x+1],
                        s[y+1, x], s[y+1, x-1], s[y, x-1], s[y-1, x-1]
                    ]
                    if sum(nb) == 1:
                        to_remove.append((y, x))
        if not to_remove:
            break
        for y, x in to_remove:
            s[y, x] = 0
    return s

def extract_medial_axis_centerline(filled_mask):
    """
    트랙 내부가 채워진 filled_mask(255-흰색, 0-검정)를 받아
    중간선만 남기는 이미지(0/255)를 반환한다.
    """
    kernel = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(filled_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel, iterations=1)

    binary = (opened > 0).astype(np.uint8)
    skeleton = zhang_suen_thinning(binary)

    dist = cv2.distanceTransform(opened, cv2.DIST_L2, 5)
    centers = np.zeros_like(skeleton)
    h, w = skeleton.shape
    for y in range(1, h-1):
        for x in range(1, w-1):
            if skeleton[y, x]:
                d0 = dist[y, x]
                keep = True
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dy == 0 and dx == 0:
                            continue
                        if skeleton[y+dy, x+dx] and dist[y+dy, x+dx] > d0:
                            keep = False
                            break
                    if not keep:
                        break
                if keep:
                    centers[y, x] = 1

    # dilate and thin to connect centers
    dilated = cv2.dilate((centers * 255).astype(np.uint8), kernel, iterations=1)
    connected = zhang_suen_thinning((dilated > 0).astype(np.uint8))
    pruned = remove_endpoints(connected, iterations=20)
    # keep largest component
    num_labels, labels = cv2.connectedComponents(pruned.astype(np.uint8), connectivity=8)
    if num_labels > 1:
        areas = [(i, (labels == i).sum()) for i in range(1, num_labels)]
        largest_label = max(areas, key=lambda t: t[1])[0]
        main_centerline = np.zeros_like(pruned, dtype=np.uint8)
        main_centerline[labels == largest_label] = 255
    else:
        main_centerline = (pruned * 255).astype(np.uint8)
    return main_centerline

# ======== 시각화 ========
def visualize_comparison(raw_img, floodfill, centerlines_dict):
    """여러 중심선 추출 방법을 비교 시각화"""
    num_methods = len(centerlines_dict)
    fig, axes = plt.subplots(2, max(2, num_methods), figsize=(5*max(2, num_methods), 10))
    if num_methods == 1:
        axes = axes.reshape(2, 1)
    axes[0, 0].imshow(raw_img, cmap='gray', origin='lower')
    axes[0, 0].set_title("Original Map", fontsize=12, fontweight='bold')
    axes[0, 0].axis("off")

    axes[0, 1].imshow(floodfill, cmap='gray', origin='lower')
    axes[0, 1].set_title("Extracted Track", fontsize=12, fontweight='bold')
    axes[0, 1].axis("off")

    for i in range(2, axes.shape[1]):
        axes[0, i].axis('off')

    for idx, (method_name, centerline_mask) in enumerate(centerlines_dict.items()):
        overlay = create_professional_overlay(floodfill, centerline_mask)
        axes[1, idx].imshow(overlay, origin='lower')
        axes[1, idx].set_title(f"{method_name}", fontsize=12, fontweight='bold')
        axes[1, idx].axis("off")
    plt.tight_layout()
    plt.savefig('centerline_comparison.png', dpi=300, bbox_inches='tight', 
                facecolor='white', edgecolor='none')
    print("비교 결과가 'centerline_comparison.png' 파일로 저장되었습니다.")

def create_professional_overlay(background, centerline_mask):
    """더 전문적인 오버레이 생성"""
    background_dim = (background * 0.7).astype(np.uint8)
    overlay = cv2.cvtColor(background_dim, cv2.COLOR_GRAY2RGB)
    centerline_thick = cv2.dilate((centerline_mask > 0).astype(np.uint8), 
                                  np.ones((2, 2), np.uint8), iterations=1)
    cyan_color = [0, 255, 255]  # 청록색 (RGB)
    overlay[centerline_thick > 0] = cyan_color
    return overlay

def visualize_individual_results(raw_img, floodfill, centerlines_dict):
    """각 방법별로 개별 이미지 저장"""
    for method_name, centerline_mask in centerlines_dict.items():
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        axes[0].imshow(raw_img, cmap='viridis', origin='lower')
        axes[0].set_title("Original Map", fontsize=12)
        axes[0].axis("off")
        axes[1].imshow(floodfill, cmap='plasma', origin='lower')
        axes[1].set_title("Track Area", fontsize=12)
        axes[1].axis("off")
        overlay = create_advanced_overlay(floodfill, centerline_mask)
        axes[2].imshow(overlay, origin='lower')
        axes[2].set_title(f"Centerline - {method_name}", fontsize=12)
        axes[2].axis("off")
        plt.tight_layout()
        filename = f"centerline_{method_name.lower().replace(' ', '_')}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        #print(f\"{method_name} 결과가 '{filename}' 파일로 저장되었습니다.\")
        plt.close()

def create_advanced_overlay(background, centerline_mask):
    """고급 오버레이 생성 - 그라디언트 효과"""
    centerline_binary = (centerline_mask > 0).astype(np.uint8)
    dist_from_centerline = cv2.distanceTransform(1 - centerline_binary, cv2.DIST_L2, 5)
    background_colored = plt.cm.gray(background / 255.0)[:, :, :3]
    centerline_heat = plt.cm.hot(np.clip(centerline_binary * 0.8 + 0.2, 0, 1))[:, :, :3]
    alpha = centerline_binary[:, :, np.newaxis] * 0.8
    result = background_colored * (1 - alpha) + centerline_heat * alpha
    return (result * 255).astype(np.uint8)

# ======== 실행 ========
if __name__ == "__main__":
    try:
        # 1. 맵 로드
        raw_img, binary, filled, yaml_info = extract_track()

        # 2. 사전 침식
        kernel = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(filled, kernel, iterations=1)

        # 3. 여러 방법으로 중심선 추출
        centerlines = {}
        centerlines["Basic Skeleton"] = extract_centerline(filled)
        centerlines["Pruned Branches"] = extract_pruned_centerline(eroded, min_branch_length=40)
        centerlines["Main Loop Only"] = extract_main_loop_only(filled)
        centerlines["Eroded Main Loop"] = extract_main_loop_after_erosion(filled, erosion_iter=2)
        # 새로 추가한 메디안 중심선
        centerlines["Medial Axis Centerline"] = extract_medial_axis_centerline(filled)

        # 4. 비교 시각화
        visualize_comparison(raw_img, filled, centerlines)
        # 5. 개별 결과 저장
        visualize_individual_results(raw_img, filled, centerlines)

        # 6. 결과 분석 출력
        print("\n=== 중심선 추출 결과 분석 ===")
        for method_name, centerline in centerlines.items():
            pixel_count = np.sum(centerline > 0)
            print(f"{method_name}: {pixel_count} pixels")

    except Exception as e:
        print(f"에러 발생: {e}")
        import traceback
        traceback.print_exc()
