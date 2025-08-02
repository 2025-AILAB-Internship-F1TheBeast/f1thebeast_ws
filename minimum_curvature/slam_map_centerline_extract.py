import numpy as np
import cv2
from PIL import Image
import matplotlib.pyplot as plt
import yaml
import os
from skimage.morphology import skeletonize, remove_small_objects, label
from skimage.measure import regionprops
from skimage.graph import route_through_array
import networkx as nx

# ======== 경로 설정 ========
map_base_path = "/home/jonghun/f1thebeast_ws/slam-maps/07.31"
map_name = "map_1753945518"
map_img_path = os.path.join(map_base_path, f"{map_name}.pgm")
map_yaml_path = os.path.join(map_base_path, f"{map_name}.yaml")

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

# ======== 중심선 추출 ========
def extract_centerline(filled_mask):
    # 1. 거리 맵 (안 써도 되지만 참고용)
    dist = cv2.distanceTransform(filled_mask, cv2.DIST_L2, 5)

    # 2. 이진화된 mask를 skeletonize로 중심선 추출
    binary_mask = (filled_mask > 0).astype(np.uint8)

    # skeletonize는 0~1 boolean 이미지 필요
    skeleton = skeletonize(binary_mask.astype(bool))

    # 3. 시각화를 위해 0~255로 변환
    centerline_mask = (skeleton.astype(np.uint8)) * 255

    return centerline_mask

# ======== 모서리 제거 ========

import numpy as np
import cv2
from skimage.morphology import skeletonize
import networkx as nx

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

    # ---- 1. 루프 보호 ----
    loops = list(nx.cycle_basis(G))
    loop_nodes = set([n for cycle in loops for n in cycle])

    # ---- 2. 끝점 찾기 ----
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

            # 루프 접촉 가지도 제거: loop_nodes 하나만 연결되면 가지
            # 루프에 닿으면 가지 끝
            if any(n in loop_nodes for n in neighbors):
                continue  # 루프에 연결된 건 남김
            else:
                # 루프에 연결되지 않은 모든 경로 제거
                to_remove.update(trace)

            for n in neighbors:
                stack.append((n, trace + [n]))

    # ---- 4. 제거 수행 ----
    cleaned = skeleton.copy()
    for y, x in to_remove:
        cleaned[y, x] = 0

    return (cleaned * 255).astype(np.uint8)




# ======== 가지 제거 ========
def extract_main_loop_only(filled_mask):
    # 스켈레톤화
    skeleton = skeletonize(filled_mask // 255)

    # 연결된 영역 라벨링
    labeled = label(skeleton)

    # 가장 넓은 영역 (area가 가장 큰 것) 선택
    props = regionprops(labeled)
    largest_label = max(props, key=lambda x: x.area).label

    # 가장 큰 루프만 남기기
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

# ======== 시각화 ========

def visualize_all(raw_img, floodfill, centerline_mask):
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    
    # 첫 번째: 원본 이미지
    axes[0].imshow(raw_img, cmap='gray', origin='lower')
    axes[0].set_title("original")
    axes[0].axis("off")

    # 두 번째: Flood Fill된 트랙 이미지
    axes[1].imshow(floodfill, cmap='gray', origin='lower')
    axes[1].set_title("Flood Fill (track only)")
    axes[1].axis("off")

    # 세 번째: floodfill 위에 centerline 빨간색으로 덧그리기
    # → floodfill을 컬러로 바꾸고 centerline 위치에 빨간색 픽셀을 넣음
    color_overlay = cv2.cvtColor(floodfill, cv2.COLOR_GRAY2RGB)
    red_mask = centerline_mask > 0
    color_overlay[red_mask] = [255, 0, 0]  # 빨간색

    axes[2].imshow(color_overlay, origin='lower')
    axes[2].set_title("centerline overlayed")
    axes[2].axis("off")

    plt.tight_layout()
    plt.show()


# ======== 실행 ========
if __name__ == "__main__":
    try:
        # 1. 맵 로드
        raw_img, binary, filled, yaml_info = extract_track()

        # 2. 사전 침식
        kernel = np.ones((3, 3), np.uint8)
        eroded = cv2.erode(filled, kernel, iterations=1)

        # 3. 중심선 추출 + 가지 제거
        centerline = extract_pruned_centerline(eroded, min_branch_length=40)

        # 4. 시각화
        visualize_all(raw_img, filled, centerline)

    except Exception as e:
        print(f"에러 발생: {e}")
        import traceback
        traceback.print_exc()



