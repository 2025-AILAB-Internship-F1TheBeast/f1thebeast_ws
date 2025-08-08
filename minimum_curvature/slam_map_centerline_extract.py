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

# ======== 개선된 시각화 ========

def visualize_comparison(raw_img, floodfill, centerlines_dict):
    """여러 중심선 추출 방법을 비교 시각화"""
    num_methods = len(centerlines_dict)
    fig, axes = plt.subplots(2, max(2, num_methods), figsize=(5*max(2, num_methods), 10))
    
    if num_methods == 1:
        axes = axes.reshape(2, 1)
    
    # 첫 번째 행: 원본과 전처리 결과
    axes[0, 0].imshow(raw_img, cmap='gray', origin='lower')
    axes[0, 0].set_title("Original Map", fontsize=12, fontweight='bold')
    axes[0, 0].axis("off")
    
    axes[0, 1].imshow(floodfill, cmap='gray', origin='lower')
    axes[0, 1].set_title("Extracted Track", fontsize=12, fontweight='bold')
    axes[0, 1].axis("off")
    
    # 나머지 첫 번째 행 subplot 숨기기
    for i in range(2, axes.shape[1]):
        axes[0, i].axis('off')
    
    # 두 번째 행: 각 방법별 중심선 결과
    for idx, (method_name, centerline_mask) in enumerate(centerlines_dict.items()):
        # 더 세련된 시각화 방법
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
    # 배경을 약간 어둡게 만들어 대비 향상
    background_dim = (background * 0.7).astype(np.uint8)
    
    # RGB로 변환
    overlay = cv2.cvtColor(background_dim, cv2.COLOR_GRAY2RGB)
    
    # 중심선에 두께 추가 (더 잘 보이도록)
    centerline_thick = cv2.dilate((centerline_mask > 0).astype(np.uint8), 
                                  np.ones((2, 2), np.uint8), iterations=1)
    
    # 중심선을 밝은 청록색으로 표시 (더 전문적)
    cyan_color = [0, 255, 255]  # 청록색 (BGR이 아닌 RGB)
    overlay[centerline_thick > 0] = cyan_color
    
    return overlay

def visualize_individual_results(raw_img, floodfill, centerlines_dict):
    """각 방법별로 개별 이미지 저장"""
    for method_name, centerline_mask in centerlines_dict.items():
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        # 원본
        axes[0].imshow(raw_img, cmap='viridis', origin='lower')
        axes[0].set_title("Original Map", fontsize=12)
        axes[0].axis("off")
        
        # 트랙만
        axes[1].imshow(floodfill, cmap='plasma', origin='lower')
        axes[1].set_title("Track Area", fontsize=12)
        axes[1].axis("off")
        
        # 중심선 오버레이
        overlay = create_advanced_overlay(floodfill, centerline_mask)
        axes[2].imshow(overlay, origin='lower')
        axes[2].set_title(f"Centerline - {method_name}", fontsize=12)
        axes[2].axis("off")
        
        plt.tight_layout()
        filename = f'centerline_{method_name.lower().replace(" ", "_")}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"{method_name} 결과가 '{filename}' 파일로 저장되었습니다.")
        plt.close()

def create_advanced_overlay(background, centerline_mask):
    """고급 오버레이 생성 - 그라디언트 효과"""
    # 거리 변환으로 두께 그라디언트 생성
    centerline_binary = (centerline_mask > 0).astype(np.uint8)
    
    # 중심선 주변에 그라디언트 효과
    dist_from_centerline = cv2.distanceTransform(
        1 - centerline_binary, cv2.DIST_L2, 5)
    
    # 배경을 컬러맵으로 변환
    background_colored = plt.cm.gray(background / 255.0)[:, :, :3]
    
    # 중심선 영역에 열지도 효과
    centerline_heat = plt.cm.hot(np.clip(centerline_binary * 0.8 + 0.2, 0, 1))[:, :, :3]
    
    # 알파 블렌딩
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
        
        # 방법 1: 기본 스켈레톤화
        centerlines["Basic Skeleton"] = extract_centerline(filled)
        
        # 방법 2: 가지 제거 (기존 사용 방법)
        centerlines["Pruned Branches"] = extract_pruned_centerline(eroded, min_branch_length=40)
        
        # 방법 3: 메인 루프만 추출
        centerlines["Main Loop Only"] = extract_main_loop_only(filled)
        
        # 방법 4: 침식 후 메인 루프
        centerlines["Eroded Main Loop"] = extract_main_loop_after_erosion(filled, erosion_iter=2)

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



