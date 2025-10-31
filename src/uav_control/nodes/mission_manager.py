#!/usr/bin/env python3
"""
Mission Manager Node
High-level mission coordination and monitoring.
Tracks mission progress, logs statistics, and handles mission completion.
"""

import rospy
from uav_msgs.msg import UAVState, TargetState, MissionStatus
from std_msgs.msg import String
import time

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=False)
        
        # Mission state
        self.uav_state = None
        self.target_state = None
        self.vision_status = None
        
        # Mission phases
        self.phases = {
            'IDLE': {'start': None, 'end': None},
            'TAKEOFF': {'start': None, 'end': None},
            'SEARCH': {'start': None, 'end': None},
            'TRACKING': {'start': None, 'end': None},
            'APPROACH': {'start': None, 'end': None},
            'ENGAGE': {'start': None, 'end': None},
            'MISSION_COMPLETE': {'start': None, 'end': None}
        }
        
        self.current_phase = 'IDLE'
        self.mission_start_time = rospy.Time.now()
        self.mission_complete = False
        
        # Statistics
        self.stats = {
            'total_detections': 0,
            'tracking_losses': 0,
            'reidentifications': 0,
            'max_altitude': 0.0,
            'total_distance': 0.0
        }
        
        # Publishers
        self.mission_status_pub = rospy.Publisher('/mission/status', MissionStatus, queue_size=10)
        self.mission_log_pub = rospy.Publisher('/mission/log', String, queue_size=10)
        
        # Subscribers
        self.uav_sub = rospy.Subscriber('/uav/state', UAVState, 
                                       self.uav_callback, queue_size=10)
        self.target_sub = rospy.Subscriber('/vision/target_state', TargetState,
                                          self.target_callback, queue_size=10)
        self.vision_sub = rospy.Subscriber('/vision/status', String,
                                          self.vision_callback, queue_size=10)
        
        # Timer for status updates
        self.status_timer = rospy.Timer(rospy.Duration(2.0), self.publish_status)
        
        rospy.loginfo("Mission Manager initialized")
        self.log_message("Mission Manager started")
    
    def uav_callback(self, msg):
        """Process UAV state updates"""
        # Track phase transitions
        if msg.flight_mode != self.current_phase:
            self.phase_transition(self.current_phase, msg.flight_mode)
            self.current_phase = msg.flight_mode
        
        # Update statistics
        if msg.pose.position.z > self.stats['max_altitude']:
            self.stats['max_altitude'] = msg.pose.position.z
        
        self.uav_state = msg
        
        # Check for mission completion
        if msg.flight_mode == 'MISSION_COMPLETE' and not self.mission_complete:
            self.mission_complete = True
            self.log_mission_summary()
    
    def target_callback(self, msg):
        """Process target state updates"""
        if msg.is_locked:
            self.stats['total_detections'] += 1
        
        if msg.is_reidentified:
            self.stats['reidentifications'] += 1
        
        if msg.frames_lost > 0:
            self.stats['tracking_losses'] += 1
        
        self.target_state = msg
    
    def vision_callback(self, msg):
        """Process vision status updates"""
        self.vision_status = msg.data
    
    def phase_transition(self, old_phase, new_phase):
        """Handle phase transitions"""
        current_time = rospy.Time.now()
        
        # End old phase
        if old_phase in self.phases and self.phases[old_phase]['start'] is not None:
            self.phases[old_phase]['end'] = current_time
            duration = (current_time - self.phases[old_phase]['start']).to_sec()
            self.log_message(f"Phase {old_phase} completed in {duration:.2f}s")
        
        # Start new phase
        if new_phase in self.phases:
            self.phases[new_phase]['start'] = current_time
            self.log_message(f"Phase {new_phase} started")
    
    def publish_status(self, event):
        """Publish mission status"""
        msg = MissionStatus()
        msg.header.stamp = rospy.Time.now()
        
        # Mission state
        msg.mission_state = self.current_phase
        
        # Calculate progress
        msg.progress = self.calculate_progress()
        
        # Phase timings
        msg.takeoff_time = self.get_phase_duration('TAKEOFF')
        msg.search_time = self.get_phase_duration('SEARCH')
        msg.tracking_time = self.get_phase_duration('TRACKING')
        msg.approach_time = self.get_phase_duration('APPROACH')
        msg.engagement_time = self.get_phase_duration('ENGAGE')
        
        # Statistics
        msg.total_detections = self.stats['total_detections']
        msg.tracking_losses = self.stats['tracking_losses']
        msg.reidentifications = self.stats['reidentifications']
        msg.mission_success = self.mission_complete
        
        self.mission_status_pub.publish(msg)
    
    def calculate_progress(self):
        """Calculate mission progress (0.0 to 1.0)"""
        phase_weights = {
            'IDLE': 0.0,
            'TAKEOFF': 0.15,
            'SEARCH': 0.35,
            'TRACKING': 0.55,
            'APPROACH': 0.80,
            'ENGAGE': 0.95,
            'MISSION_COMPLETE': 1.0
        }
        
        return phase_weights.get(self.current_phase, 0.0)
    
    def get_phase_duration(self, phase):
        """Get duration of a phase in seconds"""
        if phase not in self.phases:
            return 0.0
        
        start = self.phases[phase]['start']
        end = self.phases[phase]['end']
        
        if start is None:
            return 0.0
        
        if end is None:
            # Phase still ongoing
            return (rospy.Time.now() - start).to_sec()
        
        return (end - start).to_sec()
    
    def log_message(self, message):
        """Log a message to the mission log"""
        timestamp = rospy.Time.now()
        log_msg = String()
        log_msg.data = f"[{timestamp.to_sec():.2f}] {message}"
        self.mission_log_pub.publish(log_msg)
        rospy.loginfo(message)
    
    def log_mission_summary(self):
        """Log comprehensive mission summary"""
        total_time = (rospy.Time.now() - self.mission_start_time).to_sec()
        
        summary = "\n" + "="*60 + "\n"
        summary += "MISSION SUMMARY\n"
        summary += "="*60 + "\n"
        summary += f"Total Mission Time: {total_time:.2f} seconds\n"
        summary += "\nPhase Breakdown:\n"
        
        for phase in ['TAKEOFF', 'SEARCH', 'TRACKING', 'APPROACH', 'ENGAGE']:
            duration = self.get_phase_duration(phase)
            percentage = (duration / total_time * 100) if total_time > 0 else 0
            summary += f"  {phase:15s}: {duration:6.2f}s ({percentage:5.1f}%)\n"
        
        summary += "\nStatistics:\n"
        summary += f"  Total Detections: {self.stats['total_detections']}\n"
        summary += f"  Tracking Losses: {self.stats['tracking_losses']}\n"
        summary += f"  Re-identifications: {self.stats['reidentifications']}\n"
        summary += f"  Max Altitude: {self.stats['max_altitude']:.2f}m\n"
        
        summary += "\nMission Status: "
        summary += "SUCCESS ✓\n" if self.mission_complete else "INCOMPLETE\n"
        summary += "="*60 + "\n"
        
        rospy.loginfo(summary)
        
        # Publish to log topic
        log_msg = String()
        log_msg.data = summary
        self.mission_log_pub.publish(log_msg)
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = MissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass

