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
            self.df = pd.read_csv(self.csv_file_path)
            print(f"Data loaded successfully. Shape: {self.df.shape}")
            print(f"Columns: {list(self.df.columns)}")
        except FileNotFoundError:
            print(f"Error: File {self.csv_file_path} not found")
        except Exception as e:
            print(f"Error loading data: {e}")
    
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
        axes[0].set_ylabel('Cross Track Error (m)')
        axes[0].set_title(f'Vehicle Control Metrics Over Time {time_range_str}')
        axes[0].grid(True, alpha=0.3)
        # Add RMS text box
        axes[0].text(0.02, 0.98, f'RMS: {cross_track_rms:.6f}', 
                     transform=axes[0].transAxes, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Plot yaw error
        axes[1].plot(data['timestamp'], data['yaw_error'], 'r-', linewidth=1)
        axes[1].set_ylabel('Yaw Error (degree)')
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
        print(f"Time range: {self.df['timestamp'].min():.3f} - {self.df['timestamp'].max():.3f} seconds")
        print(f"Duration: {self.df['timestamp'].max() - self.df['timestamp'].min():.3f} seconds")
        print("\nBasic Statistics:")
        print(self.df[['cross_track_error', 'yaw_error', 'speed_error', 'current_speed']].describe())
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
    
    def plot_track_heatmap(self, segment_results, track_name="Catalunya", save_plot=False):
        """
        Plot track map with cross track error RMS as heatmap overlay
        
        Args:
            segment_results (list): Results from analyze_track_segments
            track_name (str): Name of the track (Catalunya, etc.)
            save_plot (bool): Whether to save the plot
        """
        if not segment_results:
            print("No segment results to plot")
            return
        
        # Path to track files
        map_dir = f"../map/f1tenth_racetracks/{track_name}"
        map_image_path = f"{map_dir}/{track_name}_map.png"
        centerline_path = f"{map_dir}/{track_name}_centerline.csv"
        raceline_path = f"{map_dir}/{track_name}_raceline.csv"
        
        # Check if files exist
        if not os.path.exists(map_image_path):
            print(f"Map image not found: {map_image_path}")
            return
        
        try:
            import cv2
        except ImportError:
            print("OpenCV not installed. Install with: pip install opencv-python")
            return
        
        # Load map image
        map_image = cv2.imread(map_image_path)
        if map_image is None:
            print(f"Failed to load map image: {map_image_path}")
            return
        
        map_image_rgb = cv2.cvtColor(map_image, cv2.COLOR_BGR2RGB)
        
        # Load centerline or raceline if available
        track_points = None
        if os.path.exists(centerline_path):
            try:
                centerline_df = pd.read_csv(centerline_path)
                if 'x_m' in centerline_df.columns and 'y_m' in centerline_df.columns:
                    track_points = centerline_df[['x_m', 'y_m']].values
                elif 'x' in centerline_df.columns and 'y' in centerline_df.columns:
                    track_points = centerline_df[['x', 'y']].values
            except Exception as e:
                print(f"Error loading centerline: {e}")
        
        if track_points is None and os.path.exists(raceline_path):
            try:
                raceline_df = pd.read_csv(raceline_path)
                if 'x_m' in raceline_df.columns and 'y_m' in raceline_df.columns:
                    track_points = raceline_df[['x_m', 'y_m']].values
                elif 'x' in raceline_df.columns and 'y' in raceline_df.columns:
                    track_points = raceline_df[['x', 'y']].values
            except Exception as e:
                print(f"Error loading raceline: {e}")
        
        # Extract RMS values
        cross_track_rms_values = [result['cross_track_rms'] for result in segment_results]
        min_rms = min(cross_track_rms_values)
        max_rms = max(cross_track_rms_values)
        
        # Normalize RMS values for colormap
        normalized_rms = [(rms - min_rms) / (max_rms - min_rms) if max_rms > min_rms else 0.5 
                         for rms in cross_track_rms_values]
        
        # Create figure
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # Plot 1: Original map
        ax1.imshow(map_image_rgb)
        ax1.set_title(f'{track_name} Track Map', fontsize=16, fontweight='bold')
        ax1.axis('off')
        
        # Plot 2: Map with heatmap overlay
        ax2.imshow(map_image_rgb, alpha=0.7)
        
        if track_points is not None:
            # Map track coordinates to image coordinates
            # This is a simplified mapping - you might need to adjust based on map scale/offset
            height, width = map_image_rgb.shape[:2]
            
            # Normalize track coordinates to image space
            x_coords = track_points[:, 0]
            y_coords = track_points[:, 1]
            
            # Simple linear mapping (might need adjustment for specific tracks)
            x_min, x_max = x_coords.min(), x_coords.max()
            y_min, y_max = y_coords.min(), y_coords.max()
            
            # Map to image coordinates with margins
            margin = 0.1
            x_img = ((x_coords - x_min) / (x_max - x_min)) * width * (1 - 2*margin) + width * margin
            y_img = height - (((y_coords - y_min) / (y_max - y_min)) * height * (1 - 2*margin) + height * margin)
            
            # Divide track into segments and color by RMS
            num_segments = len(segment_results)
            points_per_segment = len(track_points) // num_segments
            
            # Create colormap
            cmap = plt.cm.RdYlGn_r  # Red (high error) to Green (low error)
            
            for i, (segment_result, norm_rms) in enumerate(zip(segment_results, normalized_rms)):
                start_idx = i * points_per_segment
                end_idx = (i + 1) * points_per_segment if i < num_segments - 1 else len(track_points)
                
                if start_idx < len(x_img) and end_idx <= len(x_img):
                    segment_x = x_img[start_idx:end_idx]
                    segment_y = y_img[start_idx:end_idx]
                    
                    color = cmap(norm_rms)
                    ax2.plot(segment_x, segment_y, color=color, linewidth=8, alpha=0.8,
                            label=f'Seg {i+1}: RMS={segment_result["cross_track_rms"]:.4f}')
            
            # Add colorbar
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=min_rms, vmax=max_rms))
            sm.set_array([])
            cbar = plt.colorbar(sm, ax=ax2, shrink=0.6, aspect=20)
            cbar.set_label('Cross Track Error RMS', rotation=270, labelpad=20, fontsize=12)
        
        else:
            # If no track data, create a simple visualization
            ax2.text(0.5, 0.5, 'Track data not available\nShowing segment RMS values only', 
                    transform=ax2.transAxes, ha='center', va='center', 
                    fontsize=14, bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            # Add RMS values as text
            for i, result in enumerate(segment_results):
                y_pos = 0.9 - i * 0.05
                if y_pos > 0.1:
                    ax2.text(0.02, y_pos, f'Segment {result["segment"]}: RMS = {result["cross_track_rms"]:.4f}',
                            transform=ax2.transAxes, fontsize=10,
                            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        
        ax2.set_title(f'{track_name} Track - Cross Track Error RMS Heatmap', fontsize=16, fontweight='bold')
        ax2.axis('off')
        
        plt.tight_layout()
        
        if save_plot:
            plot_filename = f"{track_name.lower()}_track_heatmap_{len(segment_results)}segments.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"Track heatmap saved as {plot_filename}")
        
        plt.show()
        
        # Print segment summary
        print(f"\n{track_name} Track Segment Analysis:")
        print("=" * 60)
        print(f"{'Segment':<8} {'RMS':<12} {'Color Intensity':<15}")
        print("-" * 60)
        for i, (result, norm_rms) in enumerate(zip(segment_results, normalized_rms)):
            intensity = "Low" if norm_rms < 0.33 else "Medium" if norm_rms < 0.67 else "High"
            print(f"{result['segment']:<8} {result['cross_track_rms']:<12.6f} {intensity:<15}")
    
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

        # Check position columns
        position_cols = None
        if 'x' in self.df.columns and 'y' in self.df.columns:
            position_cols = ('x', 'y')
        elif 'current_x' in self.df.columns and 'current_y' in self.df.columns:
            position_cols = ('current_x', 'current_y')
        elif 'x_m' in self.df.columns and 'y_m' in self.df.columns:
            position_cols = ('x_m', 'y_m')
        elif 'pos_x' in self.df.columns and 'pos_y' in self.df.columns:
            position_cols = ('pos_x', 'pos_y')
        elif 'position_x' in self.df.columns and 'position_y' in self.df.columns:
            position_cols = ('position_x', 'position_y')

        if position_cols is None:
            print("Error: Position data not found in CSV file")
            print(f"Available columns: {list(self.df.columns)}")
            return

        print(f"Using position columns: {position_cols}")

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
        x_pos = data[position_cols[0]].values
        y_pos = data[position_cols[1]].values
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
            calculate_max_values=False
        )
        analyzer.print_results(results_filtered)
        all_results.append(results_filtered)
    else:
        # Default: Analyze 0 to 15 seconds range
        results_filtered = analyzer.analyze_metrics(start_time=0, end_time=15, calculate_max_values=False)
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
        # Plot metrics for the specific time range
        if args.start_time is not None and args.end_time is not None:
            analyzer.plot_metrics(start_time=args.start_time, end_time=args.end_time, save_plot=True, output_dir=output_dir)
        else:
            analyzer.plot_metrics(start_time=7.495, end_time=11.795, save_plot=True, output_dir=output_dir)
        
        # Plot vehicle path with error heatmaps
        print("\n" + "="*50)
        print("GENERATING VEHICLE PATH HEATMAPS")
        print("="*50)

        reference_path_file = "/home/jys/ROS2/map_creater/raceline.csv"

        if args.start_time is not None and args.end_time is not None:
            analyzer.plot_vehicle_path_heatmap(start_time=args.start_time, end_time=args.end_time, save_plot=True, output_dir=output_dir, reference_path_file=reference_path_file)
        else:
            analyzer.plot_vehicle_path_heatmap(start_time=7.495, end_time=11.795, save_plot=True, output_dir=output_dir, reference_path_file=reference_path_file)
    
    print(f"\nAnalysis completed! All files saved in: {output_dir}")


if __name__ == "__main__":
    main()