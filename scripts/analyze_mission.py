#!/usr/bin/env python3
"""
Mission Data Analysis Script
Analyzes rosbag files from mission recordings and generates statistics.
"""

import sys
import argparse
import rosbag
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt

def analyze_bag(bag_file):
    """Analyze mission bag file"""
    print(f"Analyzing bag file: {bag_file}")
    print("="*60)
    
    try:
        bag = rosbag.Bag(bag_file)
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return
    
    # Statistics
    stats = {
        'detections': [],
        'altitudes': [],
        'distances': [],
        'flight_modes': defaultdict(float),
        'track_ids': set(),
        'timestamps': []
    }
    
    start_time = None
    end_time = None
    
    # Read messages
    print("\nReading messages...")
    
    for topic, msg, t in bag.read_messages():
        if start_time is None:
            start_time = t.to_sec()
        end_time = t.to_sec()
        
        # UAV state
        if topic == '/uav/state':
            stats['altitudes'].append(msg.pose.position.z)
            if msg.distance_to_target > 0:
                stats['distances'].append(msg.distance_to_target)
            stats['flight_modes'][msg.flight_mode] += 0.05  # Assuming 20Hz
        
        # Detections
        elif topic == '/vision/detections':
            stats['detections'].append(len(msg.detections))
        
        # Target state
        elif topic == '/vision/target_state':
            if msg.is_locked:
                stats['track_ids'].add(msg.track_id)
    
    bag.close()
    
    # Calculate metrics
    total_time = end_time - start_time if end_time and start_time else 0
    
    print("\n" + "="*60)
    print("MISSION ANALYSIS RESULTS")
    print("="*60)
    
    print(f"\nTotal Mission Time: {total_time:.2f} seconds")
    
    # Flight mode breakdown
    print("\nFlight Mode Breakdown:")
    for mode, duration in sorted(stats['flight_modes'].items()):
        percentage = (duration / total_time * 100) if total_time > 0 else 0
        print(f"  {mode:20s}: {duration:6.2f}s ({percentage:5.1f}%)")
    
    # Detection statistics
    if stats['detections']:
        total_frames = len(stats['detections'])
        frames_with_detections = sum(1 for d in stats['detections'] if d > 0)
        detection_rate = frames_with_detections / total_frames * 100 if total_frames > 0 else 0
        
        print(f"\nDetection Statistics:")
        print(f"  Total frames: {total_frames}")
        print(f"  Frames with detections: {frames_with_detections}")
        print(f"  Detection rate: {detection_rate:.1f}%")
        print(f"  Unique tracks: {len(stats['track_ids'])}")
    
    # Altitude statistics
    if stats['altitudes']:
        print(f"\nAltitude Statistics:")
        print(f"  Max altitude: {max(stats['altitudes']):.2f}m")
        print(f"  Min altitude: {min(stats['altitudes']):.2f}m")
        print(f"  Mean altitude: {np.mean(stats['altitudes']):.2f}m")
    
    # Distance statistics
    if stats['distances']:
        print(f"\nDistance to Target:")
        print(f"  Max distance: {max(stats['distances']):.2f}m")
        print(f"  Min distance: {min(stats['distances']):.2f}m")
        print(f"  Mean distance: {np.mean(stats['distances']):.2f}m")
    
    print("\n" + "="*60)
    
    return stats

def plot_mission_data(stats, output_file=None):
    """Plot mission data"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Mission Analysis', fontsize=16)
    
    # Altitude over time
    if stats['altitudes']:
        axes[0, 0].plot(stats['altitudes'])
        axes[0, 0].set_title('Altitude Over Time')
        axes[0, 0].set_xlabel('Frame')
        axes[0, 0].set_ylabel('Altitude (m)')
        axes[0, 0].grid(True)
    
    # Distance to target
    if stats['distances']:
        axes[0, 1].plot(stats['distances'])
        axes[0, 1].set_title('Distance to Target')
        axes[0, 1].set_xlabel('Frame')
        axes[0, 1].set_ylabel('Distance (m)')
        axes[0, 1].grid(True)
    
    # Detections per frame
    if stats['detections']:
        axes[1, 0].plot(stats['detections'])
        axes[1, 0].set_title('Detections Per Frame')
        axes[1, 0].set_xlabel('Frame')
        axes[1, 0].set_ylabel('Number of Detections')
        axes[1, 0].grid(True)
    
    # Flight mode distribution
    if stats['flight_modes']:
        modes = list(stats['flight_modes'].keys())
        durations = list(stats['flight_modes'].values())
        axes[1, 1].bar(range(len(modes)), durations)
        axes[1, 1].set_title('Flight Mode Distribution')
        axes[1, 1].set_xlabel('Flight Mode')
        axes[1, 1].set_ylabel('Duration (s)')
        axes[1, 1].set_xticks(range(len(modes)))
        axes[1, 1].set_xticklabels(modes, rotation=45, ha='right')
        axes[1, 1].grid(True, axis='y')
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"\nPlot saved to: {output_file}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Analyze UAV mission bag files')
    parser.add_argument('bag_file', help='Path to bag file')
    parser.add_argument('--plot', action='store_true', help='Generate plots')
    parser.add_argument('--output', help='Output file for plots (PNG)')
    
    args = parser.parse_args()
    
    # Analyze bag file
    stats = analyze_bag(args.bag_file)
    
    # Generate plots if requested
    if args.plot and stats:
        try:
            plot_mission_data(stats, args.output)
        except Exception as e:
            print(f"Error generating plots: {e}")
            print("Make sure matplotlib is installed: pip3 install matplotlib")

if __name__ == '__main__':
    main()

