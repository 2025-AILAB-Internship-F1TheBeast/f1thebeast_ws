import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from pathlib import Path
import sys
import argparse

class MetricsAnalyzer:
    def __init__(self, csv_file_path):
        """
        Initialize the MetricsAnalyzer with a CSV file path
        
        Args:
            csv_file_path (str): Path to the CSV file containing metrics data
        """
        self.csv_file_path = csv_file_path
        self.df = None
        self.load_data()
    
    def load_data(self):
        """Load CSV data into pandas DataFrame"""
        try:
            # CSV 파일 로드 시 데이터 타입 명시
            dtype_dict = {
                'timestamp': float,
                'cross_track_error': float,
                'yaw_error': float,
                'speed_error': float,
                'max_cross_track_error': float,
                'max_yaw_error': float,
                'max_speed_error': float,
                'current_x': float,
                'current_y': float,
                'current_yaw': float,
                'current_speed': float,
                'target_speed': float,
                'closest_waypoint_idx': int
            }
            
            self.df = pd.read_csv(self.csv_file_path, dtype=dtype_dict)
            print(f"Data loaded successfully. Shape: {self.df.shape}")
            print(f"Columns: {list(self.df.columns)}")
            
            # target_speed 보정 (0.625배에서 실제 값으로 변환)
            if 'target_speed' in self.df.columns:
                original_target_speed_mean = self.df['target_speed'].mean()
                self.df['target_speed'] = self.df['target_speed'] / 0.625
                corrected_target_speed_mean = self.df['target_speed'].mean()
                print(f"Target speed corrected: {original_target_speed_mean:.3f} -> {corrected_target_speed_mean:.3f} (divided by 0.625)")
                
                # speed_error도 다시 계산 (target_speed가 변경되었으므로)
                if 'current_speed' in self.df.columns:
                    original_speed_error_mean = self.df['speed_error'].mean()
                    self.df['speed_error'] = np.abs(self.df['current_speed'] - self.df['target_speed'])
                    corrected_speed_error_mean = self.df['speed_error'].mean()
                    print(f"Speed error recalculated: {original_speed_error_mean:.3f} -> {corrected_speed_error_mean:.3f}")
            
            # 데이터 타입 확인
            print(f"Data types:")
            for col in ['timestamp', 'cross_track_error', 'yaw_error', 'speed_error']:
                if col in self.df.columns:
                    print(f"  {col}: {self.df[col].dtype}")
            
            # Check for required columns
            required_cols = ['timestamp', 'cross_track_error', 'yaw_error', 'speed_error']
            missing_cols = [col for col in required_cols if col not in self.df.columns]
            if missing_cols:
                print(f"Warning: Missing required columns: {missing_cols}")
            else:
                print("All required columns found!")
                
            # NaN 값 확인 및 처리
            nan_counts = self.df.isnull().sum()
            if nan_counts.sum() > 0:
                print(f"Warning: Found NaN values:")
                for col, count in nan_counts.items():
                    if count > 0:
                        print(f"  {col}: {count} NaN values")
                
                # NaN 값이 있는 행 제거
                original_len = len(self.df)
                self.df = self.df.dropna()
                removed_rows = original_len - len(self.df)
                if removed_rows > 0:
                    print(f"Removed {removed_rows} rows with NaN values")
                    
        except FileNotFoundError:
            print(f"Error: File {self.csv_file_path} not found")
        except ValueError as e:
            print(f"Error loading data - Value Error: {e}")
            print("Attempting to load without dtype specification...")
            try:
                # 타입 지정 없이 다시 시도
                self.df = pd.read_csv(self.csv_file_path)
                print(f"Data loaded successfully without dtype. Shape: {self.df.shape}")
                
                # 수동으로 타입 변환 시도
                numeric_cols = ['timestamp', 'cross_track_error', 'yaw_error', 'speed_error', 
                            'max_cross_track_error', 'max_yaw_error', 'max_speed_error',
                            'current_x', 'current_y', 'current_yaw', 'current_speed', 'target_speed']
                
                for col in numeric_cols:
                    if col in self.df.columns:
                        try:
                            self.df[col] = pd.to_numeric(self.df[col], errors='coerce')
                            print(f"Converted {col} to numeric")
                        except Exception as convert_error:
                            print(f"Failed to convert {col} to numeric: {convert_error}")
                
                # target_speed 보정 (0.625배에서 실제 값으로 변환)
                if 'target_speed' in self.df.columns:
                    original_target_speed_mean = self.df['target_speed'].mean()
                    self.df['target_speed'] = self.df['target_speed'] / 0.625
                    corrected_target_speed_mean = self.df['target_speed'].mean()
                    print(f"Target speed corrected: {original_target_speed_mean:.3f} -> {corrected_target_speed_mean:.3f} (divided by 0.625)")
                    
                    # speed_error도 다시 계산 (target_speed가 변경되었으므로)
                    if 'current_speed' in self.df.columns:
                        original_speed_error_mean = self.df['speed_error'].mean()
                        self.df['speed_error'] = np.abs(self.df['current_speed'] - self.df['target_speed'])
                        corrected_speed_error_mean = self.df['speed_error'].mean()
                        print(f"Speed error recalculated: {original_speed_error_mean:.3f} -> {corrected_speed_error_mean:.3f}")
                
                # closest_waypoint_idx를 정수로 변환
                if 'closest_waypoint_idx' in self.df.columns:
                    try:
                        self.df['closest_waypoint_idx'] = pd.to_numeric(self.df['closest_waypoint_idx'], errors='coerce').astype('Int64')
                        print("Converted closest_waypoint_idx to integer")
                    except Exception as convert_error:
                        print(f"Failed to convert closest_waypoint_idx to integer: {convert_error}")
                
                # 변환 후 NaN 값 확인
                nan_counts = self.df.isnull().sum()
                if nan_counts.sum() > 0:
                    print(f"Warning: Found NaN values after conversion:")
                    for col, count in nan_counts.items():
                        if count > 0:
                            print(f"  {col}: {count} NaN values")
                    
                    # NaN 값이 있는 행 제거
                    original_len = len(self.df)
                    self.df = self.df.dropna()
                    removed_rows = original_len - len(self.df)
                    if removed_rows > 0:
                        print(f"Removed {removed_rows} rows with NaN values after conversion")
                
                print("Manual type conversion completed")
                
            except Exception as fallback_error:
                print(f"Error loading data even without dtype: {fallback_error}")
                
        except Exception as e:
            print(f"Error loading data: {e}")
            import traceback
            traceback.print_exc()
    
    def calculate_rms(self, values):
        """
        Calculate Root Mean Square (RMS) of values
        
        Args:
            values (array-like): Array of values
            
        Returns:
            float: RMS value
        """
        return np.sqrt(np.mean(values**2))
    
    def calculate_max(self, values):
        """
        Calculate maximum absolute value
        
        Args:
            values (array-like): Array of values
            
        Returns:
            float: Maximum absolute value
        """
        return np.max(np.abs(values))
    
    def filter_by_timestamp(self, start_time, end_time):
        """
        Filter data by timestamp range
        
        Args:
            start_time (float): Start timestamp
            end_time (float): End timestamp
            
        Returns:
            DataFrame: Filtered data
        """
        if self.df is None:
            print("No data loaded")
            return None
        
        filtered_df = self.df[(self.df['timestamp'] >= start_time) & 
                             (self.df['timestamp'] <= end_time)]
        return filtered_df
    
    def analyze_metrics(self, start_time=None, end_time=None, calculate_max_values=False):
        """
        Analyze metrics (RMS and optionally max values) for given time range
        
        Args:
            start_time (float, optional): Start timestamp. If None, use all data
            end_time (float, optional): End timestamp. If None, use all data
            calculate_max_values (bool): Whether to calculate max values
            
        Returns:
            dict: Dictionary containing analysis results
        """
        if self.df is None:
            print("No data loaded")
            return None
        
        # Filter data if time range is specified
        if start_time is not None and end_time is not None:
            data = self.filter_by_timestamp(start_time, end_time)
            print(f"Analyzing data from {start_time} to {end_time} seconds")
        else:
            data = self.df
            print("Analyzing all data")
        
        if data is None or len(data) == 0:
            print("No data in specified time range")
            return None
        
        # Calculate RMS values
        cross_track_rms = self.calculate_rms(data['cross_track_error'])
        yaw_error_rms = self.calculate_rms(data['yaw_error'])
        speed_error_rms = self.calculate_rms(data['speed_error'])
        
        results = {
            'data_points': len(data),
            'time_range': (start_time, end_time) if start_time is not None else 'All data',
            'cross_track_error_rms': cross_track_rms,
            'yaw_error_rms': yaw_error_rms,
            'speed_error_rms': speed_error_rms
        }
        
        # Calculate max values if requested
        if calculate_max_values:
            cross_track_max = self.calculate_max(data['cross_track_error'])
            yaw_error_max = self.calculate_max(data['yaw_error'])
            speed_error_max = self.calculate_max(data['speed_error'])
            
            results.update({
                'cross_track_error_max': cross_track_max,
                'yaw_error_max': yaw_error_max,
                'speed_error_max': speed_error_max
            })
        print(f"Analysis results: {results}")
        return results
    
    def print_results(self, results):
        """
        Print analysis results in a formatted way
        
        Args:
            results (dict): Results dictionary from analyze_metrics
        """
        if results is None:
            return
        
        print("\n" + "="*50)
        print("METRICS ANALYSIS RESULTS")
        print("="*50)
        print(f"Data points analyzed: {results['data_points']}")
        print(f"Time range: {results['time_range']}")
        print("-"*50)
        print("RMS VALUES:")
        print(f"  Cross Track Error RMS: {results['cross_track_error_rms']:.6f}")
        print(f"  Yaw Error RMS: {results['yaw_error_rms']:.6f}")
        print(f"  Speed Error RMS: {results['speed_error_rms']:.6f}")
        
        if 'cross_track_error_max' in results:
            print("-"*50)
            print("MAX VALUES:")
            print(f"  Cross Track Error Max: {results['cross_track_error_max']:.6f}")
            print(f"  Yaw Error Max: {results['yaw_error_max']:.6f}")
            print(f"  Speed Error Max: {results['speed_error_max']:.6f}")
        
        print("="*50)
    
    def plot_metrics(self, start_time=None, end_time=None, save_plot=False, output_dir=None):
        """
        Plot metrics over time with RMS values displayed
        
        Args:
            start_time (float, optional): Start timestamp for plotting
            end_time (float, optional): End timestamp for plotting
            save_plot (bool): Whether to save the plot
            output_dir (str, optional): Directory to save the plot
        """
        if self.df is None:
            print("No data loaded")
            return
        
        # Filter data if time range is specified
        if start_time is not None and end_time is not None:
            data = self.filter_by_timestamp(start_time, end_time)
            time_range_str = f"({start_time:.1f}s - {end_time:.1f}s)"
        else:
            data = self.df
            time_range_str = "(All data)"
        
        if data is None or len(data) == 0:
            print("No data to plot")
            return
        
        # Calculate RMS values for display
        cross_track_rms = self.calculate_rms(data['cross_track_error'])
        yaw_error_rms = self.calculate_rms(data['yaw_error'])
        speed_error_rms = self.calculate_rms(data['speed_error'])
        
        # Create subplots
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        
        # Plot cross track error
        axes[0].plot(data['timestamp'], data['cross_track_error'], 'b-', linewidth=1)
        axes[0].set_ylabel('Cross Track Error')
        axes[0].set_title(f'Vehicle Control Metrics Over Time {time_range_str}')
        axes[0].grid(True, alpha=0.3)
        # Add RMS text box
        axes[0].text(0.02, 0.98, f'RMS: {cross_track_rms:.6f}', 
                     transform=axes[0].transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Plot yaw error
        axes[1].plot(data['timestamp'], data['yaw_error'], 'r-', linewidth=1)
        axes[1].set_ylabel('Yaw Error (rad)')
        axes[1].grid(True, alpha=0.3)
        # Add RMS text box
        axes[1].text(0.02, 0.98, f'RMS: {yaw_error_rms:.6f}', 
                     transform=axes[1].transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        
        # Plot speed error
        axes[2].plot(data['timestamp'], data['speed_error'], 'g-', linewidth=1)
        axes[2].set_ylabel('Speed Error (m/s)')
        axes[2].set_xlabel('Time (s)')
        axes[2].grid(True, alpha=0.3)
        # Add RMS text box
        axes[2].text(0.02, 0.98, f'RMS: {speed_error_rms:.6f}', 
                     transform=axes[2].transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        plt.tight_layout()
        
        if save_plot:
            if start_time is not None and end_time is not None:
                plot_filename = f"metrics_plot_{start_time:.1f}_{end_time:.1f}s.png"
            else:
                plot_filename = "metrics_plot_all_data.png"
            
            if output_dir:
                plot_filename = os.path.join(output_dir, plot_filename)
            
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Plot saved as {plot_filename}")
        
        plt.show()
    
    def get_data_summary(self):
        """Get basic summary statistics of the data"""
        if self.df is None:
            print("No data loaded")
            return
        
        print("\n" + "="*50)
        print("DATA SUMMARY")
        print("="*50)
        print(f"Total data points: {len(self.df)}")
        
        # timestamp 컬럼 타입 확인 및 처리
        if 'timestamp' in self.df.columns:
            try:
                # timestamp가 숫자형인지 확인
                if pd.api.types.is_numeric_dtype(self.df['timestamp']):
                    time_min = self.df['timestamp'].min()
                    time_max = self.df['timestamp'].max()
                    duration = time_max - time_min
                    print(f"Time range: {time_min:.3f} - {time_max:.3f} seconds")
                    print(f"Duration: {duration:.3f} seconds")
                else:
                    print(f"Timestamp column type: {self.df['timestamp'].dtype}")
                    print("Converting timestamp to numeric...")
                    # 숫자로 변환 시도
                    self.df['timestamp'] = pd.to_numeric(self.df['timestamp'], errors='coerce')
                    if not self.df['timestamp'].isnull().all():
                        time_min = self.df['timestamp'].min()
                        time_max = self.df['timestamp'].max()
                        duration = time_max - time_min
                        print(f"Time range: {time_min:.3f} - {time_max:.3f} seconds")
                        print(f"Duration: {duration:.3f} seconds")
                    else:
                        print("Failed to convert timestamp to numeric")
            except Exception as e:
                print(f"Error processing timestamp: {e}")
                print(f"Timestamp sample values: {self.df['timestamp'].head()}")
        else:
            print("Timestamp column not found")
        
        # Check if speed columns exist
        speed_cols = []
        if 'current_speed' in self.df.columns:
            speed_cols.append('current_speed')
        if 'target_speed' in self.df.columns:
            speed_cols.append('target_speed')
            
        summary_cols = ['cross_track_error', 'yaw_error', 'speed_error'] + speed_cols
        available_cols = [col for col in summary_cols if col in self.df.columns]
        
        if available_cols:
            print("\nBasic Statistics:")
            try:
                print(self.df[available_cols].describe())
            except Exception as e:
                print(f"Error generating statistics: {e}")
                # 개별 컬럼별로 시도
                for col in available_cols:
                    try:
                        print(f"\n{col} statistics:")
                        print(self.df[col].describe())
                    except Exception as col_error:
                        print(f"  Error with {col}: {col_error}")
        else:
            print("No valid columns found for statistics")
        
        print("="*50)
    
    def save_analysis_to_csv(self, results_list, output_filename="analysis_results.csv"):
        """
        Save analysis results to CSV file
        
        Args:
            results_list (list): List of results dictionaries from analyze_metrics
            output_filename (str): Full path to the output CSV file
        """
        if not results_list:
            print("No results to save")
            return
        
        # Prepare data for CSV
        csv_data = []
        
        for i, results in enumerate(results_list):
            if results is None:
                continue
                
            row = {
                'analysis_id': i + 1,
                'data_points': results['data_points'],
                'start_time': results['time_range'][0] if isinstance(results['time_range'], tuple) else 'All',
                'end_time': results['time_range'][1] if isinstance(results['time_range'], tuple) else 'All',
                'cross_track_error_rms': results['cross_track_error_rms'],
                'yaw_error_rms': results['yaw_error_rms'],
                'speed_error_rms': results['speed_error_rms']
            }
            
            # Add max values if they exist
            if 'cross_track_error_max' in results:
                row.update({
                    'cross_track_error_max': results['cross_track_error_max'],
                    'yaw_error_max': results['yaw_error_max'],
                    'speed_error_max': results['speed_error_max']
                })
            else:
                row.update({
                    'cross_track_error_max': 'N/A',
                    'yaw_error_max': 'N/A',
                    'speed_error_max': 'N/A'
                })
            
            csv_data.append(row)
        
        # Create DataFrame and save to CSV
        if csv_data:
            df_results = pd.DataFrame(csv_data)
            df_results.to_csv(output_filename, index=False)
            print(f"Analysis results saved to: {output_filename}")
            
            # Also print a summary table
            print("\nAnalysis Results Summary:")
            print("-" * 100)
            print(df_results.to_string(index=False))
        else:
            print("No valid results to save")
    
    def analyze_track_segments(self, num_segments=10):
        """
        Analyze cross track error RMS for different track segments
        
        Args:
            num_segments (int): Number of segments to divide the track into
            
        Returns:
            dict: Dictionary containing segment analysis results
        """
        if self.df is None:
            print("No data loaded")
            return None
        
        total_duration = self.df['timestamp'].max() - self.df['timestamp'].min()
        segment_duration = total_duration / num_segments
        start_time = self.df['timestamp'].min()
        
        segment_results = []
        
        for i in range(num_segments):
            segment_start = start_time + i * segment_duration
            segment_end = start_time + (i + 1) * segment_duration
            
            segment_data = self.filter_by_timestamp(segment_start, segment_end)
            
            if segment_data is not None and len(segment_data) > 0:
                cross_track_rms = self.calculate_rms(segment_data['cross_track_error'])
                yaw_error_rms = self.calculate_rms(segment_data['yaw_error'])
                speed_error_rms = self.calculate_rms(segment_data['speed_error'])
                
                segment_results.append({
                    'segment': i + 1,
                    'start_time': segment_start,
                    'end_time': segment_end,
                    'duration': segment_duration,
                    'data_points': len(segment_data),
                    'cross_track_rms': cross_track_rms,
                    'yaw_error_rms': yaw_error_rms,
                    'speed_error_rms': speed_error_rms
                })
        
        return segment_results
    
    def plot_segment_analysis(self, segment_results, save_plot=False):
        """
        Plot RMS values for each track segment
        
        Args:
            segment_results (list): Results from analyze_track_segments
            save_plot (bool): Whether to save the plot
        """
        if not segment_results:
            print("No segment results to plot")
            return
        
        segments = [result['segment'] for result in segment_results]
        cross_track_rms_values = [result['cross_track_rms'] for result in segment_results]
        yaw_error_rms_values = [result['yaw_error_rms'] for result in segment_results]
        speed_error_rms_values = [result['speed_error_rms'] for result in segment_results]
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Cross Track Error RMS by segment (main focus)
        axes[0, 0].bar(segments, cross_track_rms_values, color='skyblue', alpha=0.7, edgecolor='navy')
        axes[0, 0].set_title('Cross Track Error RMS by Track Segment', fontsize=14, fontweight='bold')
        axes[0, 0].set_xlabel('Track Segment')
        axes[0, 0].set_ylabel('Cross Track Error RMS')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_xticks(segments)
        
        # Add value labels on bars
        for i, v in enumerate(cross_track_rms_values):
            axes[0, 0].text(i + 1, v + max(cross_track_rms_values) * 0.01, f'{v:.4f}', 
                           ha='center', va='bottom', fontsize=9)
        
        # Plot 2: Yaw Error RMS by segment
        axes[0, 1].bar(segments, yaw_error_rms_values, color='lightcoral', alpha=0.7, edgecolor='darkred')
        axes[0, 1].set_title('Yaw Error RMS by Track Segment', fontsize=14)
        axes[0, 1].set_xlabel('Track Segment')
        axes[0, 1].set_ylabel('Yaw Error RMS (rad)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].set_xticks(segments)
        
        # Plot 3: Speed Error RMS by segment
        axes[1, 0].bar(segments, speed_error_rms_values, color='lightgreen', alpha=0.7, edgecolor='darkgreen')
        axes[1, 0].set_title('Speed Error RMS by Track Segment', fontsize=14)
        axes[1, 0].set_xlabel('Track Segment')
        axes[1, 0].set_ylabel('Speed Error RMS (m/s)')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].set_xticks(segments)
        
        # Plot 4: Combined line plot
        axes[1, 1].plot(segments, cross_track_rms_values, 'o-', label='Cross Track Error', linewidth=2, markersize=6)
        axes[1, 1].plot(segments, yaw_error_rms_values, 's-', label='Yaw Error', linewidth=2, markersize=6)
        axes[1, 1].plot(segments, speed_error_rms_values, '^-', label='Speed Error', linewidth=2, markersize=6)
        axes[1, 1].set_title('All RMS Values by Track Segment', fontsize=14)
        axes[1, 1].set_xlabel('Track Segment')
        axes[1, 1].set_ylabel('RMS Value')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
        axes[1, 1].set_xticks(segments)
        
        plt.tight_layout()
        
        if save_plot:
            plot_filename = f"track_segments_analysis_{len(segments)}segments.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Segment analysis plot saved as {plot_filename}")
        
        plt.show()
    
    def print_segment_results(self, segment_results):
        """
        Print segment analysis results in a formatted table
        
        Args:
            segment_results (list): Results from analyze_track_segments
        """
        if not segment_results:
            print("No segment results to display")
            return
        
        print("\n" + "="*80)
        print("TRACK SEGMENT ANALYSIS RESULTS")
        print("="*80)
        print(f"{'Seg':<3} {'Start':<6} {'End':<6} {'Duration':<8} {'Points':<7} {'Cross RMS':<10} {'Yaw RMS':<10} {'Speed RMS':<10}")
        print("-"*80)
        
        for result in segment_results:
            print(f"{result['segment']:<3} "
                  f"{result['start_time']:<6.1f} "
                  f"{result['end_time']:<6.1f} "
                  f"{result['duration']:<8.2f} "
                  f"{result['data_points']:<7} "
                  f"{result['cross_track_rms']:<10.6f} "
                  f"{result['yaw_error_rms']:<10.6f} "
                  f"{result['speed_error_rms']:<10.6f}")
        
        print("="*80)
        
        # Summary statistics
        cross_track_values = [r['cross_track_rms'] for r in segment_results]
        print(f"Cross Track Error RMS - Min: {min(cross_track_values):.6f}, "
              f"Max: {max(cross_track_values):.6f}, "
              f"Mean: {np.mean(cross_track_values):.6f}, "
              f"Std: {np.std(cross_track_values):.6f}")
    
    def plot_vehicle_path_heatmap(self, start_time=None, end_time=None, save_plot=False, output_dir=None, reference_path_file=None):
        """
        Plot vehicle path with error metrics as heatmap overlay and reference path as dashed line

        Args:
            start_time (float, optional): Start timestamp for plotting
            end_time (float, optional): End timestamp for plotting
            save_plot (bool): Whether to save the plot
            output_dir (str, optional): Directory to save the plot
            reference_path_file (str, optional): Path to reference raceline CSV
        """
        if self.df is None:
            print("No data loaded")
            return

        # Check for position columns in the uploaded CSV format
        if 'current_x' not in self.df.columns or 'current_y' not in self.df.columns:
            print("Error: Position data (current_x, current_y) not found in CSV file")
            print(f"Available columns: {list(self.df.columns)}")
            return

        print("Using position columns: current_x, current_y")

        # Filter data if time range is specified
        if start_time is not None and end_time is not None:
            data = self.filter_by_timestamp(start_time, end_time)
            time_range_str = f"({start_time:.1f}s - {end_time:.1f}s)"
        else:
            data = self.df
            time_range_str = "(All data)"

        if data is None or len(data) == 0:
            print("No data to plot")
            return

        # Extract position and error data
        x_pos = data['current_x'].values
        y_pos = data['current_y'].values
        cross_track_error = data['cross_track_error'].values
        yaw_error = data['yaw_error'].values
        speed_error = data['speed_error'].values

        print(f"Plotting {len(x_pos)} data points")
        print(f"X range: {x_pos.min():.2f} to {x_pos.max():.2f}")
        print(f"Y range: {y_pos.min():.2f} to {y_pos.max():.2f}")

        # Load reference path if provided
        ref_x, ref_y = None, None
        if reference_path_file and os.path.exists(reference_path_file):
            try:
                print(f"Loading reference path from: {reference_path_file}")
                
                # 수동으로 헤더 이름 지정 (CSV 파일의 주석에서 확인한 구조)
                header_names = ['s_m', 'x_m', 'y_m', 'psi_rad', 'kappa_radpm', 'vx_mps', 'ax_mps2']
                
                # CSV 파일 읽기 (헤더 없이 읽고 수동으로 헤더 지정)
                ref_df = pd.read_csv(reference_path_file, 
                                    sep=';',  # 세미콜론 구분자
                                    comment='#',  # # 주석 행 무시
                                    header=None,  # 헤더 없음
                                    names=header_names)  # 수동으로 헤더 이름 지정
        
                print(f"Reference CSV columns: {list(ref_df.columns)}")
                print(f"Reference CSV shape: {ref_df.shape}")
        
                if 'x_m' in ref_df.columns and 'y_m' in ref_df.columns:
                    ref_x = ref_df['x_m'].values
                    ref_y = ref_df['y_m'].values
                    print(f"Reference path loaded: {len(ref_x)} points")
                    print(f"Reference X range: {ref_x.min():.2f} to {ref_x.max():.2f}")
                    print(f"Reference Y range: {ref_y.min():.2f} to {ref_y.max():.2f}")
                else:
                    print(f"Expected columns 'x_m', 'y_m' not found in reference file")
                    print(f"Available columns: {list(ref_df.columns)}")
            except Exception as e:
                print(f"Failed to load reference path: {e}")
                import traceback
                traceback.print_exc()

        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))

        # Plot 1: Vehicle path only + reference path
        axes[0, 0].plot(x_pos, y_pos, 'b-', linewidth=2, alpha=0.7, label='Vehicle Path')
        axes[0, 0].scatter(x_pos[0], y_pos[0], color='green', s=100, marker='o', label='Start', zorder=5)
        axes[0, 0].scatter(x_pos[-1], y_pos[-1], color='red', s=100, marker='s', label='End', zorder=5)
        if ref_x is not None and ref_y is not None:
            print("Adding reference path to plot 1")
            axes[0, 0].plot(ref_x, ref_y, 'k--', linewidth=2, alpha=0.7, label='Reference Path')
        axes[0, 0].set_title(f'Vehicle Path {time_range_str}', fontsize=14, fontweight='bold')
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 0].axis('equal')

        # Plot 2: Cross Track Error Heatmap
        scatter2 = axes[0, 1].scatter(x_pos, y_pos, c=np.abs(cross_track_error), 
                                     cmap='Reds', s=20, alpha=0.8)
        axes[0, 1].plot(x_pos, y_pos, 'k-', linewidth=1, alpha=0.3)
        if ref_x is not None and ref_y is not None:
            axes[0, 1].plot(ref_x, ref_y, 'k--', linewidth=2, alpha=0.7, label='Reference Path')
        axes[0, 1].set_title('Cross Track Error Heatmap', fontsize=14, fontweight='bold')
        axes[0, 1].set_xlabel('X Position (m)')
        axes[0, 1].set_ylabel('Y Position (m)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].axis('equal')
        cbar2 = plt.colorbar(scatter2, ax=axes[0, 1], shrink=0.8)
        cbar2.set_label('|Cross Track Error|', rotation=270, labelpad=15)

        # Plot 3: Yaw Error Heatmap
        scatter3 = axes[1, 0].scatter(x_pos, y_pos, c=np.abs(yaw_error), 
                                     cmap='Blues', s=20, alpha=0.8)
        axes[1, 0].plot(x_pos, y_pos, 'k-', linewidth=1, alpha=0.3)
        if ref_x is not None and ref_y is not None:
            axes[1, 0].plot(ref_x, ref_y, 'k--', linewidth=2, alpha=0.7, label='Reference Path')
        axes[1, 0].set_title('Yaw Error Heatmap', fontsize=14, fontweight='bold')
        axes[1, 0].set_xlabel('X Position (m)')
        axes[1, 0].set_ylabel('Y Position (m)')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].axis('equal')
        cbar3 = plt.colorbar(scatter3, ax=axes[1, 0], shrink=0.8)
        cbar3.set_label('|Yaw Error| (rad)', rotation=270, labelpad=15)

        # Plot 4: Speed Error Heatmap
        scatter4 = axes[1, 1].scatter(x_pos, y_pos, c=np.abs(speed_error), 
                                     cmap='Greens', s=20, alpha=0.8)
        axes[1, 1].plot(x_pos, y_pos, 'k-', linewidth=1, alpha=0.3)
        if ref_x is not None and ref_y is not None:
            axes[1, 1].plot(ref_x, ref_y, 'k--', linewidth=2, alpha=0.7, label='Reference Path')
        axes[1, 1].set_title('Speed Error Heatmap', fontsize=14, fontweight='bold')
        axes[1, 1].set_xlabel('X Position (m)')
        axes[1, 1].set_ylabel('Y Position (m)')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].axis('equal')
        cbar4 = plt.colorbar(scatter4, ax=axes[1, 1], shrink=0.8)
        cbar4.set_label('|Speed Error| (m/s)', rotation=270, labelpad=15)

        plt.tight_layout()

        if save_plot:
            if start_time is not None and end_time is not None:
                plot_filename = f"vehicle_path_heatmap_{start_time:.1f}_{end_time:.1f}s.png"
            else:
                plot_filename = "vehicle_path_heatmap_all_data.png"
            if output_dir:
                plot_filename = os.path.join(output_dir, plot_filename)
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Vehicle path heatmap saved as {plot_filename}")

        plt.show()

        # Print path statistics
        path_length = np.sum(np.sqrt(np.diff(x_pos)**2 + np.diff(y_pos)**2))
        print(f"\nVehicle Path Statistics {time_range_str}:")
        print("=" * 60)
        print(f"Total path length: {path_length:.2f} m")
        print(f"Start position: ({x_pos[0]:.2f}, {y_pos[0]:.2f})")
        print(f"End position: ({x_pos[-1]:.2f}, {y_pos[-1]:.2f})")
        print(f"Data points: {len(x_pos)}")

        # Error statistics along the path
        print(f"\nError Statistics along Path:")
        print("-" * 40)
        print(f"Cross Track Error - Max: {np.max(np.abs(cross_track_error)):.4f}, Mean: {np.mean(np.abs(cross_track_error)):.4f}")
        print(f"Yaw Error - Max: {np.max(np.abs(yaw_error)):.4f}, Mean: {np.mean(np.abs(yaw_error)):.4f}")
        print(f"Speed Error - Max: {np.max(np.abs(speed_error)):.4f}, Mean: {np.mean(np.abs(speed_error)):.4f}")
    
    def plot_speed_analysis(self, start_time=None, end_time=None, save_plot=False, output_dir=None):
        """
        Plot speed analysis with current_speed vs target_speed
        
        Args:
            start_time (float, optional): Start timestamp for plotting
            end_time (float, optional): End timestamp for plotting
            save_plot (bool): Whether to save the plot
            output_dir (str, optional): Directory to save the plot
        """
        if self.df is None:
            print("No data loaded")
            return
        
        # Check if speed columns exist
        if 'current_speed' not in self.df.columns or 'target_speed' not in self.df.columns:
            print("Error: Speed data (current_speed, target_speed) not found in CSV file")
            return
        
        # Filter data if time range is specified
        if start_time is not None and end_time is not None:
            data = self.filter_by_timestamp(start_time, end_time)
            time_range_str = f"({start_time:.1f}s - {end_time:.1f}s)"
        else:
            data = self.df
            time_range_str = "(All data)"
        
        if data is None or len(data) == 0:
            print("No data to plot")
            return
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: Speed vs Time
        axes[0, 0].plot(data['timestamp'], data['current_speed'], 'b-', linewidth=2, label='Current Speed', alpha=0.8)
        axes[0, 0].plot(data['timestamp'], data['target_speed'], 'r--', linewidth=2, label='Target Speed', alpha=0.8)
        axes[0, 0].set_title(f'Speed Tracking {time_range_str}', fontsize=14, fontweight='bold')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Speed (m/s)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        
        # Plot 2: Speed Error vs Time
        axes[0, 1].plot(data['timestamp'], data['speed_error'], 'g-', linewidth=2)
        axes[0, 1].axhline(y=0, color='k', linestyle='-', alpha=0.3)
        speed_error_rms = self.calculate_rms(data['speed_error'])
        axes[0, 1].set_title(f'Speed Error (RMS: {speed_error_rms:.3f})', fontsize=14, fontweight='bold')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Speed Error (m/s)')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Current vs Target Speed Scatter
        axes[1, 0].scatter(data['target_speed'], data['current_speed'], alpha=0.6, s=10)
        min_speed = min(data['target_speed'].min(), data['current_speed'].min())
        max_speed = max(data['target_speed'].max(), data['current_speed'].max())
        axes[1, 0].plot([min_speed, max_speed], [min_speed, max_speed], 'r--', linewidth=2, label='Perfect Tracking')
        axes[1, 0].set_title('Current vs Target Speed', fontsize=14, fontweight='bold')
        axes[1, 0].set_xlabel('Target Speed (m/s)')
        axes[1, 0].set_ylabel('Current Speed (m/s)')
        axes[1, 0].grid(True, alpha=0.3)
        axes[1, 0].legend()
        axes[1, 0].axis('equal')
        
        # Plot 4: Speed Error Histogram
        axes[1, 1].hist(data['speed_error'], bins=50, alpha=0.7, color='green', edgecolor='black')
        axes[1, 1].axvline(x=0, color='red', linestyle='--', linewidth=2, label='Zero Error')
        axes[1, 1].axvline(x=data['speed_error'].mean(), color='orange', linestyle='-', linewidth=2, label=f'Mean: {data["speed_error"].mean():.3f}')
        axes[1, 1].set_title('Speed Error Distribution', fontsize=14, fontweight='bold')
        axes[1, 1].set_xlabel('Speed Error (m/s)')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend()
        
        plt.tight_layout()
        
        if save_plot:
            if start_time is not None and end_time is not None:
                plot_filename = f"speed_analysis_{start_time:.1f}_{end_time:.1f}s.png"
            else:
                plot_filename = "speed_analysis_all_data.png"
            if output_dir:
                plot_filename = os.path.join(output_dir, plot_filename)
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Speed analysis plot saved as {plot_filename}")
        
        plt.show()
        
        # Print speed statistics
        print(f"\nSpeed Analysis Statistics {time_range_str}:")
        print("=" * 60)
        print(f"Target Speed - Min: {data['target_speed'].min():.2f}, Max: {data['target_speed'].max():.2f}, Mean: {data['target_speed'].mean():.2f}")
        print(f"Current Speed - Min: {data['current_speed'].min():.2f}, Max: {data['current_speed'].max():.2f}, Mean: {data['current_speed'].mean():.2f}")
        print(f"Speed Error - Min: {data['speed_error'].min():.3f}, Max: {data['speed_error'].max():.3f}, Mean: {data['speed_error'].mean():.3f}")
        print(f"Speed Error RMS: {speed_error_rms:.3f}")
        print(f"Speed Error Std: {data['speed_error'].std():.3f}")
    
def main():
    """Main function to demonstrate usage"""
    
    # Command line argument parsing
    parser = argparse.ArgumentParser(description='Analyze vehicle control metrics from CSV file')
    parser.add_argument('csv_file', 
                       help='Path to the CSV file containing metrics data')
    parser.add_argument('--track', '-t', 
                       default='Catalunya', 
                       help='Track name for heatmap visualization (default: Catalunya)')
    parser.add_argument('--start-time', 
                       type=float, 
                       help='Start time for specific analysis (optional)')
    parser.add_argument('--end-time', 
                       type=float, 
                       help='End time for specific analysis (optional)')
    parser.add_argument('--no-plots', 
                       action='store_true', 
                       help='Skip plot generation')
    parser.add_argument('--segments', 
                       type=int, 
                       default=10, 
                       help='Number of segments for track analysis (default: 10)')
    
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.csv_file):
        print(f"Error: CSV file not found: {args.csv_file}")
        print("Please provide a valid path to the CSV file")
        sys.exit(1)
    
    # Create output directory based on CSV filename
    csv_basename = os.path.splitext(os.path.basename(args.csv_file))[0]  # Remove .csv extension
    analysis_base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "analysis")
    output_dir = os.path.join(analysis_base_dir, f"analysis_{csv_basename}")
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Analyzing CSV file: {args.csv_file}")
    print(f"Track: {args.track}")
    print(f"Output directory: {output_dir}")
    
    # Create analyzer instance
    analyzer = MetricsAnalyzer(args.csv_file)
    
    # Get data summary
    analyzer.get_data_summary()
    
    # Store results for CSV export
    all_results = []
    
    # Analyze specific time range if provided
    if args.start_time is not None and args.end_time is not None:
        print(f"\nAnalyzing specific time range: {args.start_time} - {args.end_time} seconds")
        results_filtered = analyzer.analyze_metrics(
            start_time=args.start_time, 
            end_time=args.end_time, 
            calculate_max_values=True
        )
        analyzer.print_results(results_filtered)
        all_results.append(results_filtered)
    
    # Analyze all data with max values
    results_all = analyzer.analyze_metrics(calculate_max_values=True)
    analyzer.print_results(results_all)
    all_results.append(results_all)
    
    # Analyze first half vs second half
    total_duration = analyzer.df['timestamp'].max() - analyzer.df['timestamp'].min()
    mid_time = analyzer.df['timestamp'].min() + total_duration / 2
    
    results_first_half = analyzer.analyze_metrics(
        start_time=analyzer.df['timestamp'].min(), 
        end_time=mid_time, 
        calculate_max_values=True
    )
    analyzer.print_results(results_first_half)
    all_results.append(results_first_half)
    
    results_second_half = analyzer.analyze_metrics(
        start_time=mid_time, 
        end_time=analyzer.df['timestamp'].max(), 
        calculate_max_values=True
    )
    analyzer.print_results(results_second_half)
    all_results.append(results_second_half)
    
    # Track segment analysis (텍스트 출력만)
    print("\n" + "="*50)
    print(f"TRACK SEGMENT ANALYSIS ({args.segments} segments)")
    print("="*50)
    segment_results = analyzer.analyze_track_segments(num_segments=args.segments)
    if segment_results:
        analyzer.print_segment_results(segment_results)
    
    # Save all analysis results to CSV in output directory
    try:
        timestamp_str = csv_basename.split('_')[-2]
        if not timestamp_str.isdigit():
            timestamp_str = "analysis"
    except:
        timestamp_str = "analysis"
    
    output_filename = os.path.join(output_dir, f"analysis_results_{timestamp_str}.csv")
    analyzer.save_analysis_to_csv(all_results, output_filename)
    
    # Plot generation (skip if --no-plots flag is used)
    if not args.no_plots:
        # 첫 번째 figure: Plot metrics (cross track error, yaw error, speed error over time)
        if args.start_time is not None and args.end_time is not None:
            analyzer.plot_metrics(start_time=args.start_time, end_time=args.end_time, save_plot=True, output_dir=output_dir)
        else:
            analyzer.plot_metrics(save_plot=True, output_dir=output_dir)
        
        # 네 번째 figure: Plot vehicle path with error heatmaps
        print("\n" + "="*50)
        print("GENERATING VEHICLE PATH HEATMAPS")
        print("="*50)

        reference_path_file = "/home/jys/ROS2/f1thebeast_ws/src/control/map/f1tenth_racetracks/Catalunya/Catalunya_raceline.csv"

        if args.start_time is not None and args.end_time is not None:
            analyzer.plot_vehicle_path_heatmap(start_time=args.start_time, end_time=args.end_time, save_plot=True, output_dir=output_dir, reference_path_file=reference_path_file)
        else:
            analyzer.plot_vehicle_path_heatmap(save_plot=True, output_dir=output_dir, reference_path_file=reference_path_file)
    
    print(f"\nAnalysis completed! All files saved in: {output_dir}")


if __name__ == "__main__":
    main()