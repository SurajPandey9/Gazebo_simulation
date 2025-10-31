#!/usr/bin/env python3
"""
Vision Manager Node
Coordinates detection and tracking, publishes unified vision state.
Manages re-identification and provides high-level vision status.
"""

import rospy
from uav_msgs.msg import DetectionArray, TargetState, MissionStatus
from std_msgs.msg import String
from geometry_msgs.msg import Point

class VisionManager:
    def __init__(self):
        rospy.init_node('vision_manager', anonymous=False)
        
        # State
        self.current_detections = None
        self.current_target = None
        self.mission_start_time = None
        
        # Statistics
        self.total_detections = 0
        self.total_frames = 0
        self.tracking_losses = 0
        self.reidentifications = 0
        self.last_track_id = -1
        
        # Publishers
        self.status_pub = rospy.Publisher('/vision/status', String, queue_size=10)
        self.mission_status_pub = rospy.Publisher('/mission/status', MissionStatus, queue_size=10)
        
        # Subscribers
        self.detection_sub = rospy.Subscriber('/vision/detections', DetectionArray,
                                              self.detection_callback, queue_size=10)
        self.target_sub = rospy.Subscriber('/vision/target_state', TargetState,
                                          self.target_callback, queue_size=10)
        
        # Timer for status updates
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        self.mission_start_time = rospy.Time.now()
        
        rospy.loginfo("Vision Manager initialized")
    
    def detection_callback(self, msg):
        """Process detection updates"""
        self.current_detections = msg
        self.total_frames += 1
        
        if len(msg.detections) > 0:
            self.total_detections += len(msg.detections)
    
    def target_callback(self, msg):
        """Process target state updates"""
        # Check for tracking loss
        if self.current_target is not None:
            if msg.track_id != self.last_track_id and self.last_track_id != -1:
                self.tracking_losses += 1
                rospy.logwarn(f"Track ID changed: {self.last_track_id} -> {msg.track_id}")
        
        # Check for re-identification
        if msg.is_reidentified:
            self.reidentifications += 1
            rospy.loginfo(f"Re-identification detected for track {msg.track_id}")
        
        self.current_target = msg
        self.last_track_id = msg.track_id
    
    def publish_status(self, event):
        """Publish vision system status"""
        # Simple status message
        if self.current_target is not None and self.current_target.is_locked:
            status = f"TRACKING: ID {self.current_target.track_id}, " \
                    f"Confidence {self.current_target.confidence:.2f}, " \
                    f"Frames {self.current_target.frames_tracked}"
        elif self.current_detections is not None and len(self.current_detections.detections) > 0:
            status = f"DETECTING: {len(self.current_detections.detections)} objects"
        else:
            status = "SEARCHING: No detections"
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
        # Mission status
        mission_msg = MissionStatus()
        mission_msg.header.stamp = rospy.Time.now()
        
        if self.current_target is not None:
            mission_msg.mission_state = "TRACKING" if self.current_target.is_locked else "SEARCHING"
        else:
            mission_msg.mission_state = "SEARCHING"
        
        # Calculate mission time
        mission_time = (rospy.Time.now() - self.mission_start_time).to_sec()
        
        # Statistics
        mission_msg.total_detections = self.total_detections
        mission_msg.tracking_losses = self.tracking_losses
        mission_msg.reidentifications = self.reidentifications
        
        # Progress estimate (simplified)
        if mission_msg.mission_state == "TRACKING":
            mission_msg.progress = 0.7
        elif self.total_detections > 0:
            mission_msg.progress = 0.4
        else:
            mission_msg.progress = 0.2
        
        self.mission_status_pub.publish(mission_msg)
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        manager = VisionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass

