#!/usr/bin/env python3
"""
Performance Monitor for Low-Spec Systems

Monitors system resources and ROS performance metrics.
Provides warnings when approaching hardware limits.

Usage:
    rosrun uav_simulation performance_monitor.py
"""

import rospy
import psutil
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from uav_msgs.msg import MissionStatus, UAVState
import json


class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('performance_monitor', anonymous=False)
        
        # Parameters
        self.monitor_rate = rospy.get_param('~monitor_rate', 1.0)  # Hz
        self.log_to_file = rospy.get_param('~log_to_file', True)
        self.log_file = rospy.get_param('~log_file', '/workspace/performance.log')
        
        # Warning thresholds
        self.max_memory_mb = rospy.get_param('~max_memory_warning', 6000)  # MB
        self.max_cpu_percent = rospy.get_param('~max_cpu_warning', 90)  # %
        self.min_fps = rospy.get_param('~min_fps_warning', 2.0)  # FPS
        
        # Performance metrics
        self.metrics = {
            'cpu_percent': 0.0,
            'memory_mb': 0.0,
            'memory_percent': 0.0,
            'camera_fps': 0.0,
            'detection_fps': 0.0,
            'sim_real_time_factor': 0.0,
            'mission_phase': 'UNKNOWN',
            'warnings': []
        }
        
        # FPS tracking
        self.camera_msg_count = 0
        self.camera_last_time = time.time()
        self.camera_fps = 0.0
        
        # Subscribers
        self.camera_sub = rospy.Subscriber(
            '/camera/rgb/image_raw',
            Image,
            self.camera_callback,
            queue_size=1
        )
        
        self.mission_sub = rospy.Subscriber(
            '/mission/status',
            MissionStatus,
            self.mission_callback,
            queue_size=1
        )
        
        self.uav_sub = rospy.Subscriber(
            '/uav/state',
            UAVState,
            self.uav_callback,
            queue_size=1
        )
        
        # Publisher
        self.status_pub = rospy.Publisher(
            '/performance/status',
            String,
            queue_size=1
        )
        
        # Log file
        if self.log_to_file:
            self.log_handle = open(self.log_file, 'w')
            self.log_handle.write("timestamp,cpu_percent,memory_mb,memory_percent,camera_fps,mission_phase\n")
        
        rospy.loginfo("Performance Monitor started")
        rospy.loginfo(f"Monitoring at {self.monitor_rate} Hz")
        rospy.loginfo(f"Logging to: {self.log_file}")
        
    def camera_callback(self, msg):
        """Track camera FPS"""
        self.camera_msg_count += 1
        
    def mission_callback(self, msg):
        """Track mission status"""
        self.metrics['mission_phase'] = msg.current_phase
        
    def uav_callback(self, msg):
        """Track UAV state"""
        pass
        
    def update_metrics(self):
        """Update all performance metrics"""
        
        # CPU usage
        self.metrics['cpu_percent'] = psutil.cpu_percent(interval=0.1)
        
        # Memory usage
        mem = psutil.virtual_memory()
        self.metrics['memory_mb'] = mem.used / (1024 * 1024)
        self.metrics['memory_percent'] = mem.percent
        
        # Camera FPS
        current_time = time.time()
        elapsed = current_time - self.camera_last_time
        if elapsed >= 1.0:
            self.camera_fps = self.camera_msg_count / elapsed
            self.metrics['camera_fps'] = self.camera_fps
            self.camera_msg_count = 0
            self.camera_last_time = current_time
        
        # Check for warnings
        self.check_warnings()
        
    def check_warnings(self):
        """Check if any metrics exceed thresholds"""
        self.metrics['warnings'] = []
        
        # Memory warning
        if self.metrics['memory_mb'] > self.max_memory_mb:
            warning = f"HIGH MEMORY: {self.metrics['memory_mb']:.0f} MB (limit: {self.max_memory_mb} MB)"
            self.metrics['warnings'].append(warning)
            rospy.logwarn(warning)
        
        # CPU warning
        if self.metrics['cpu_percent'] > self.max_cpu_percent:
            warning = f"HIGH CPU: {self.metrics['cpu_percent']:.1f}% (limit: {self.max_cpu_percent}%)"
            self.metrics['warnings'].append(warning)
            rospy.logwarn(warning)
        
        # FPS warning
        if self.camera_fps > 0 and self.camera_fps < self.min_fps:
            warning = f"LOW FPS: {self.camera_fps:.1f} FPS (minimum: {self.min_fps} FPS)"
            self.metrics['warnings'].append(warning)
            rospy.logwarn(warning)
        
    def log_metrics(self):
        """Log metrics to file"""
        if self.log_to_file:
            timestamp = time.time()
            line = f"{timestamp},{self.metrics['cpu_percent']:.1f},{self.metrics['memory_mb']:.0f},"
            line += f"{self.metrics['memory_percent']:.1f},{self.metrics['camera_fps']:.1f},"
            line += f"{self.metrics['mission_phase']}\n"
            self.log_handle.write(line)
            self.log_handle.flush()
    
    def publish_status(self):
        """Publish performance status"""
        status = {
            'cpu_percent': round(self.metrics['cpu_percent'], 1),
            'memory_mb': round(self.metrics['memory_mb'], 0),
            'memory_percent': round(self.metrics['memory_percent'], 1),
            'camera_fps': round(self.metrics['camera_fps'], 1),
            'mission_phase': self.metrics['mission_phase'],
            'warnings': self.metrics['warnings']
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
    def print_status(self):
        """Print status to console"""
        print("\n" + "="*60)
        print("PERFORMANCE MONITOR")
        print("="*60)
        print(f"CPU Usage:      {self.metrics['cpu_percent']:6.1f}%")
        print(f"Memory Usage:   {self.metrics['memory_mb']:6.0f} MB ({self.metrics['memory_percent']:.1f}%)")
        print(f"Camera FPS:     {self.metrics['camera_fps']:6.1f}")
        print(f"Mission Phase:  {self.metrics['mission_phase']}")
        
        if self.metrics['warnings']:
            print("\nWARNINGS:")
            for warning in self.metrics['warnings']:
                print(f"  ⚠️  {warning}")
        else:
            print("\n✅ All metrics within normal range")
        
        print("="*60)
        
    def run(self):
        """Main monitoring loop"""
        rate = rospy.Rate(self.monitor_rate)
        
        while not rospy.is_shutdown():
            try:
                # Update metrics
                self.update_metrics()
                
                # Log to file
                self.log_metrics()
                
                # Publish status
                self.publish_status()
                
                # Print to console
                self.print_status()
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error in performance monitor: {e}")
                
        # Cleanup
        if self.log_to_file:
            self.log_handle.close()
            rospy.loginfo(f"Performance log saved to: {self.log_file}")


if __name__ == '__main__':
    try:
        monitor = PerformanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass

