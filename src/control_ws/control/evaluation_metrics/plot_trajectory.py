import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_vehicle_trajectory(csv_file_path):
    """
    CSV 파일에서 차량 궤적 데이터를 읽어와서 시각화
    
    Args:
        csv_file_path (str): CSV 파일 경로
    """
    # CSV 파일 읽기
    try:
        df = pd.read_csv(csv_file_path)
        print(f"데이터 로드 완료: {len(df)} 개의 데이터 포인트")
    except Exception as e:
        print(f"CSV 파일 읽기 실패: {e}")
        return
    
    # 필요한 컬럼 확인
    required_columns = ['current_x', 'current_y', 'current_yaw', 'timestamp']
    for col in required_columns:
        if col not in df.columns:
            print(f"필수 컬럼 '{col}'이 없습니다.")
            return
    
    # Reference path 읽기
    reference_path = "/home/jys/ROS2/f1thebeast_ws/src/control/map/f1tenth_racetracks/Catalunya/Catalunya_raceline.csv"
    try:
        ref_df = pd.read_csv(reference_path, comment='#', sep=';', 
                           names=['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2'])
        print(f"Reference path 로드 완료: {len(ref_df)} 개의 waypoint")
    except Exception as e:
        print(f"Reference path 읽기 실패: {e}")
        ref_df = None
    
    # 데이터 추출
    x = df['current_x'].values
    y = df['current_y'].values
    yaw = df['current_yaw'].values
    timestamp = df['timestamp'].values
    
    # 플롯 생성 (단일 플롯)
    fig, ax = plt.subplots(1, 1, figsize=(14, 12))
    
    # Reference path 그리기 (있는 경우)
    if ref_df is not None:
        ref_x = ref_df['x_m'].values
        ref_y = ref_df['y_m'].values
        ref_psi = ref_df['psi_rad'].values
        
        # Reference path의 모든 waypoint들에 대해 방향 표시
        for i in range(len(ref_x)):
            # 삼각형 마커로 방향 표시 (모든 포인트)
            ax.scatter(ref_x[i], ref_y[i], marker=(3, 0, np.degrees(ref_psi[i])-90), 
                      s=15, color='black', alpha=0.7)
    
    # 차량 궤적을 파란색 점으로 표시
    ax.scatter(x, y, color='blue', s=15, alpha=0.8, label='Vehicle Positions')
    
    # 차량 헤딩 방향 화살촉 추가 (일부만 샘플링)
    step = max(1, len(x) // 50)
    sample_indices = range(0, len(x), step)
    
    for i in sample_indices:
        # 방향만 표시하는 삼각형 마커 (빨간색)
        ax.scatter(x[i], y[i], marker=(3, 0, np.degrees(yaw[i])-90), 
                  s=50, color='red', alpha=0.8)
    
    # 시작점과 끝점을 특별히 표시
    ax.scatter(x[0], y[0], color='green', s=200, marker='o', label='Start', 
              zorder=5, edgecolors='darkgreen', linewidth=2)
    ax.scatter(x[-1], y[-1], color='red', s=200, marker='s', label='End', 
              zorder=5, edgecolors='darkred', linewidth=2)
    
    # 범례에 표시할 더미 요소 추가
    ax.scatter([], [], marker=(3, 0, 0), s=15, color='black', alpha=0.7, label='Reference Direction')
    ax.scatter([], [], marker=(3, 0, 0), s=50, color='red', alpha=0.8, label='Vehicle Direction')
    
    ax.set_title('Vehicle Trajectory vs Reference Path', fontsize=16, fontweight='bold')
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=12)
    ax.axis('equal')
    
    plt.tight_layout()
    
    # 통계 정보 출력
    print("\n=== Trajectory Statistics ===")
    print(f"총 주행 시간: {timestamp[-1] - timestamp[0]:.2f} 초")
    print(f"총 데이터 포인트: {len(x)}")
    print(f"X 범위: {x.min():.2f} ~ {x.max():.2f} m")
    print(f"Y 범위: {y.min():.2f} ~ {y.max():.2f} m")
    print(f"시작 위치: ({x[0]:.2f}, {y[0]:.2f})")
    print(f"종료 위치: ({x[-1]:.2f}, {y[-1]:.2f})")
    
    # 총 주행 거리 계산
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    total_distance = np.sum(distances)
    print(f"총 주행 거리: {total_distance:.2f} m")
    
    if 'current_speed' in df.columns:
        avg_speed = df['current_speed'].mean()
        max_speed = df['current_speed'].max()
        print(f"평균 속도: {avg_speed:.2f} m/s")
        print(f"최대 속도: {max_speed:.2f} m/s")
    
    if 'cross_track_error' in df.columns:
        avg_cte = np.mean(np.abs(df['cross_track_error']))
        max_cte = np.max(np.abs(df['cross_track_error']))
        print(f"평균 Cross Track Error: {avg_cte:.6f} m")
        print(f"최대 Cross Track Error: {max_cte:.6f} m")
    
    if ref_df is not None:
        print(f"Reference path waypoints: {len(ref_df)}")
    
    plt.show()

# 메인 실행 부분
if __name__ == "__main__":
    # CSV 파일 경로 설정
    csv_file_path = "/home/jys/ROS2/f1thebeast_ws/src/control/evaluation_metrics/csv_file/9.4_Catalunya_metrics_20250804_112839.csv"
    
    # 궤적 시각화
    plot_vehicle_trajectory(csv_file_path)