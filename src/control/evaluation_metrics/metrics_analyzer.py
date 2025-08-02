import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse

class MetricsAnalyzer:
    def __init__(self, csv_file_path):
        """
        CSV 파일에서 metrics 데이터를 로드하고 분석하는 클래스
        
        Args:
            csv_file_path (str): CSV 파일 경로
        """
        self.csv_file_path = csv_file_path
        self.df = None
        self.load_data()
        
    def load_data(self):
        """CSV 파일에서 데이터 로드 (timestamp 3초 미만 제외)"""
        try:
            # 원본 데이터 로드
            df_raw = pd.read_csv(self.csv_file_path)
            print(f"📁 원본 데이터 로드: {len(df_raw)} 개의 데이터 포인트")
            
            # timestamp가 3초 이상인 데이터만 필터링
            self.df = df_raw[df_raw['timestamp'] >= 3.0].copy()
            
            # 인덱스 리셋
            self.df.reset_index(drop=True, inplace=True)
            
            print(f"✅ 필터링된 데이터 로드: {len(self.df)} 개의 데이터 포인트")
            print(f"🗑️  제외된 데이터: {len(df_raw) - len(self.df)} 개 (timestamp < 3.0초)")
            print(f"📊 분석 시간 범위: {self.df['timestamp'].min():.2f}초 ~ {self.df['timestamp'].max():.2f}초")
            
            if len(self.df) == 0:
                print("⚠️  경고: 3초 이후 데이터가 없습니다!")
                
        except Exception as e:
            print(f"❌ 데이터 로드 실패: {e}")
            
    def calculate_rms_error(self, column):
        """RMS (Root Mean Square) Error 계산"""
        return np.sqrt(np.mean(self.df[column] ** 2))
    
    def calculate_max_error(self, column):
        """Maximum Absolute Error 계산"""
        return np.max(np.abs(self.df[column]))
    
    def calculate_mean_error(self, column):
        """Mean Absolute Error 계산"""
        return np.mean(np.abs(self.df[column]))
    
    def calculate_std_error(self, column):
        """Standard Deviation 계산"""
        return np.std(self.df[column])
    
    def analyze_metrics(self):
        """전체 metrics 분석"""
        metrics = ['cross_track_error', 'yaw_error', 'speed_error']
        results = {}
        
        print("\n" + "="*80)
        print("🔍 EVALUATION METRICS ANALYSIS")
        print("="*80)
        
        for metric in metrics:
            if metric in self.df.columns:
                rms = self.calculate_rms_error(metric)
                max_err = self.calculate_max_error(metric)
                mean_err = self.calculate_mean_error(metric)
                std_err = self.calculate_std_error(metric)
                
                results[metric] = {
                    'RMS Error': rms,
                    'Max Error': max_err,
                    'Mean Error': mean_err,
                    'Std Deviation': std_err
                }
                
                print(f"\n📈 {metric.upper().replace('_', ' ')}")
                print(f"   RMS Error      : {rms:.6f}")
                print(f"   Max Error      : {max_err:.6f}")
                print(f"   Mean Error     : {mean_err:.6f}")
                print(f"   Std Deviation  : {std_err:.6f}")
                
        return results
    
    def create_summary_table(self, results):
        """결과를 표 형태로 정리"""
        summary_df = pd.DataFrame(results).T
        summary_df = summary_df.round(6)
        
        print("\n" + "="*80)
        print("📋 SUMMARY TABLE")
        print("="*80)
        print(summary_df.to_string())
        
        # CSV로 저장
        output_path = Path(self.csv_file_path).parent / "metrics_summary.csv"
        summary_df.to_csv(output_path)
        print(f"\n💾 Summary saved to: {output_path}")
        
        return summary_df
    
    def plot_metrics_over_time(self, save_plots=True):
        """시간에 따른 metrics 변화 플롯"""
        metrics = ['cross_track_error', 'yaw_error', 'speed_error']
        
        # 한글 폰트 설정 (선택사항)
        plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial', 'sans-serif']
        plt.rcParams['axes.unicode_minus'] = False
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle('Performance Metrics Over Time', fontsize=16, fontweight='bold')
        
        colors = ['red', 'blue', 'green']
        labels = ['Cross Track Error (m)', 'Yaw Error (degree)', 'Speed Error (m/s)']
        
        for i, (metric, color, label) in enumerate(zip(metrics, colors, labels)):
            if metric in self.df.columns:
                axes[i].plot(self.df['timestamp'], self.df[metric], 
                           color=color, linewidth=1.5, alpha=0.8)
                axes[i].set_ylabel(label, fontweight='bold')
                axes[i].grid(True, alpha=0.3)
                axes[i].set_title(f'{label} - RMS: {self.calculate_rms_error(metric):.4f}, Max: {self.calculate_max_error(metric):.4f}')
        
        axes[-1].set_xlabel('Time (seconds)', fontweight='bold')
        plt.tight_layout()
        
        if save_plots:
            output_path = Path(self.csv_file_path).parent / "metrics_time_plot.png"
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"📊 Time plot saved to: {output_path}")
        
        plt.show()
    
    def plot_metrics_distribution(self, save_plots=True):
        """Metrics 분포 히스토그램"""
        metrics = ['cross_track_error', 'yaw_error', 'speed_error']
        
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle('Metrics Distribution', fontsize=16, fontweight='bold')
        
        colors = ['red', 'blue', 'green']
        labels = ['Cross Track Error (m)', 'Yaw Error (degree)', 'Speed Error (m/s)']
        
        for i, (metric, color, label) in enumerate(zip(metrics, colors, labels)):
            if metric in self.df.columns:
                axes[i].hist(self.df[metric], bins=50, color=color, alpha=0.7, edgecolor='black', linewidth=0.5)
                axes[i].set_xlabel(label, fontweight='bold')
                axes[i].set_ylabel('Frequency')
                axes[i].grid(True, alpha=0.3)
                axes[i].axvline(self.calculate_mean_error(metric), color='darkred', linestyle='--', 
                              label=f'Mean: {self.calculate_mean_error(metric):.4f}')
                axes[i].legend()
        
        plt.tight_layout()
        
        if save_plots:
            output_path = Path(self.csv_file_path).parent / "metrics_distribution.png"
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"📊 Distribution plot saved to: {output_path}")
        
        plt.show()
    
    def plot_trajectory_analysis(self, save_plots=True):
        """차량 궤적과 target vs actual 비교"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Trajectory and Performance Analysis', fontsize=16, fontweight='bold')
        
        # 1. 차량 궤적
        scatter = axes[0,0].scatter(self.df['current_x'], self.df['current_y'], 
                                  c=self.df['cross_track_error'], cmap='coolwarm', s=2)
        axes[0,0].set_xlabel('X Position (m)')
        axes[0,0].set_ylabel('Y Position (m)')
        axes[0,0].set_title('Vehicle Trajectory (colored by Cross Track Error)')
        plt.colorbar(scatter, ax=axes[0,0], label='Cross Track Error (m)')
        axes[0,0].grid(True, alpha=0.3)
        axes[0,0].axis('equal')
        
        # 2. 속도 비교
        axes[0,1].plot(self.df['timestamp'], self.df['current_speed'], label='Actual Speed', color='blue', linewidth=1.5)
        axes[0,1].plot(self.df['timestamp'], self.df['target_speed'], label='Target Speed', color='red', linewidth=1.5)
        axes[0,1].set_xlabel('Time (s)')
        axes[0,1].set_ylabel('Speed (m/s)')
        axes[0,1].set_title('Speed Tracking Performance')
        axes[0,1].legend()
        axes[0,1].grid(True, alpha=0.3)
        
        # 3. Cross track error 시간 변화
        axes[1,0].plot(self.df['timestamp'], self.df['cross_track_error'], color='red', linewidth=1.5)
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Cross Track Error (m)')
        axes[1,0].set_title('Cross Track Error Over Time')
        axes[1,0].grid(True, alpha=0.3)
        
        # 4. Yaw error 시간 변화
        axes[1,1].plot(self.df['timestamp'], np.degrees(self.df['yaw_error']), color='blue', linewidth=1.5)
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Yaw Error (degrees)')
        axes[1,1].set_title('Yaw Error Over Time')
        axes[1,1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_plots:
            output_path = Path(self.csv_file_path).parent / "trajectory_analysis.png"
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            print(f"📊 Trajectory analysis saved to: {output_path}")
        
        plt.show()
    
    def generate_report(self):
        """전체 리포트 생성"""
        print("\n🚀 GENERATING COMPREHENSIVE ANALYSIS REPORT...")
        
        # 1. 기본 통계 분석
        results = self.analyze_metrics()
        
        # 2. 요약 표 생성
        summary_df = self.create_summary_table(results)
        
        # 3. 플롯 생성
        self.plot_metrics_over_time()
        self.plot_metrics_distribution()
        self.plot_trajectory_analysis()
        
        # 4. 성능 평가
        self.evaluate_performance(results)
        
        return results, summary_df
    
    def evaluate_performance(self, results):
        """성능 평가 및 등급 매기기"""
        print("\n" + "="*80)
        print("🎯 PERFORMANCE EVALUATION")
        print("="*80)
        
        # 성능 기준 (예시)
        thresholds = {
            'cross_track_error': {'excellent': 0.05, 'good': 0.1, 'fair': 0.2},
            'yaw_error': {'excellent': 0.05, 'good': 0.1, 'fair': 0.2},
            'speed_error': {'excellent': 0.5, 'good': 1.0, 'fair': 2.0}
        }
        
        def get_grade(error, thresholds):
            if error <= thresholds['excellent']:
                return "🏆 Excellent"
            elif error <= thresholds['good']:
                return "🥈 Good"
            elif error <= thresholds['fair']:
                return "🥉 Fair"
            else:
                return "❌ Needs Improvement"
        
        for metric, values in results.items():
            rms_error = values['RMS Error']
            max_error = values['Max Error']
            
            if metric in thresholds:
                rms_grade = get_grade(rms_error, thresholds[metric])
                max_grade = get_grade(max_error, thresholds[metric])
                
                print(f"\n📊 {metric.upper().replace('_', ' ')}")
                print(f"   RMS Error Grade: {rms_grade} ({rms_error:.6f})")
                print(f"   Max Error Grade: {max_grade} ({max_error:.6f})")

def main():
    parser = argparse.ArgumentParser(description='Analyze F1Tenth racing metrics')
    parser.add_argument('csv_file', help='Path to the metrics CSV file')
    parser.add_argument('--no-plots', action='store_true', help='Skip generating plots')
    
    args = parser.parse_args()
    
    # CSV 파일 존재 확인
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"❌ Error: File {csv_path} not found!")
        return
    
    # 분석기 생성 및 실행
    analyzer = MetricsAnalyzer(str(csv_path))
    
    if args.no_plots:
        results = analyzer.analyze_metrics()
        analyzer.create_summary_table(results)
        analyzer.evaluate_performance(results)
    else:
        analyzer.generate_report()

if __name__ == "__main__":
    main()