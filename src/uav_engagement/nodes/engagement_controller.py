#!/usr/bin/env python3
"""
Engagement Controller Node
Handles final target engagement logic and weapon system simulation.
Implements the EngageTarget service and manages engagement sequence.
"""

import rospy
from uav_msgs.msg import TargetState, UAVState
from uav_msgs.srv import EngageTarget, EngageTargetResponse
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
import math

class EngagementController:
    def __init__(self):
        rospy.init_node('engagement_controller', anonymous=False)
        
        # Parameters
        self.engagement_altitude = rospy.get_param('~engagement_altitude', 2.0)
        self.engagement_radius = rospy.get_param('~engagement_radius', 1.0)  # meters
        self.min_lock_time = rospy.get_param('~min_lock_time', 2.0)  # seconds
        self.centering_tolerance = rospy.get_param('~centering_tolerance', 50)  # pixels
        
        # State
        self.uav_state = None
        self.target_state = None
        self.engagement_ready = False
        self.engagement_complete = False
        self.lock_start_time = None
        self.target_engaged = False
        
        # Image dimensions (for centering check)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = self.image_width / 2
        self.image_center_y = self.image_height / 2
        
        # Publishers
        self.engagement_status_pub = rospy.Publisher('/engagement/status', String, queue_size=10)
        self.engagement_ready_pub = rospy.Publisher('/engagement/ready', Bool, queue_size=10)
        self.weapon_fire_pub = rospy.Publisher('/engagement/fire', Bool, queue_size=10)
        
        # Subscribers
        self.uav_sub = rospy.Subscriber('/uav/state', UAVState, 
                                       self.uav_callback, queue_size=10)
        self.target_sub = rospy.Subscriber('/vision/target_state', TargetState,
                                          self.target_callback, queue_size=10)
        
        # Service
        self.engage_service = rospy.Service('/engagement/engage_target', 
                                           EngageTarget, 
                                           self.handle_engage_request)
        
        # Timer for engagement checks
        self.check_timer = rospy.Timer(rospy.Duration(0.1), self.check_engagement_conditions)
        
        rospy.loginfo("Engagement Controller initialized")
        rospy.loginfo(f"Engagement parameters: altitude={self.engagement_altitude}m, "
                     f"radius={self.engagement_radius}m, lock_time={self.min_lock_time}s")
    
    def uav_callback(self, msg):
        """Process UAV state updates"""
        self.uav_state = msg
    
    def target_callback(self, msg):
        """Process target state updates"""
        self.target_state = msg
        
        # Reset lock timer if target is lost
        if not msg.is_locked and self.lock_start_time is not None:
            rospy.logwarn("Target lock lost, resetting engagement timer")
            self.lock_start_time = None
            self.engagement_ready = False
    
    def check_engagement_conditions(self, event):
        """Check if engagement conditions are met"""
        if self.uav_state is None or self.target_state is None:
            self.engagement_ready = False
            self.publish_status("WAITING: No UAV or target data")
            return
        
        # Check if target is locked
        if not self.target_state.is_locked:
            self.engagement_ready = False
            self.lock_start_time = None
            self.publish_status("WAITING: Target not locked")
            return
        
        # Check altitude
        current_altitude = self.uav_state.pose.position.z
        if current_altitude > self.engagement_altitude + 0.5:
            self.engagement_ready = False
            self.publish_status(f"DESCENDING: Altitude {current_altitude:.1f}m > {self.engagement_altitude}m")
            return
        
        # Check if target is centered
        if not self.is_target_centered():
            self.engagement_ready = False
            self.lock_start_time = None
            self.publish_status("CENTERING: Target not centered in frame")
            return
        
        # Check distance to target (if available)
        if self.uav_state.distance_to_target > 0:
            if self.uav_state.distance_to_target > self.engagement_radius * 2:
                self.engagement_ready = False
                self.publish_status(f"APPROACHING: Distance {self.uav_state.distance_to_target:.1f}m")
                return
        
        # Start lock timer if not started
        if self.lock_start_time is None:
            self.lock_start_time = rospy.Time.now()
            rospy.loginfo("Target lock acquired, starting engagement timer")
        
        # Check lock duration
        lock_duration = (rospy.Time.now() - self.lock_start_time).to_sec()
        
        if lock_duration >= self.min_lock_time:
            if not self.engagement_ready:
                rospy.loginfo(f"Engagement conditions met! Lock time: {lock_duration:.1f}s")
                self.engagement_ready = True
                
                # Publish ready status
                ready_msg = Bool()
                ready_msg.data = True
                self.engagement_ready_pub.publish(ready_msg)
            
            self.publish_status(f"READY: Lock stable for {lock_duration:.1f}s")
        else:
            time_remaining = self.min_lock_time - lock_duration
            self.publish_status(f"LOCKING: {time_remaining:.1f}s remaining")
    
    def is_target_centered(self):
        """Check if target is centered in the image"""
        if self.target_state is None:
            return False
        
        # Calculate target center
        target_center_x = self.target_state.bbox_x + self.target_state.bbox_width / 2
        target_center_y = self.target_state.bbox_y + self.target_state.bbox_height / 2
        
        # Calculate error from image center
        error_x = abs(target_center_x - self.image_center_x)
        error_y = abs(target_center_y - self.image_center_y)
        
        # Check if within tolerance
        return error_x < self.centering_tolerance and error_y < self.centering_tolerance
    
    def handle_engage_request(self, req):
        """Handle engagement service request"""
        response = EngageTargetResponse()
        
        if self.target_engaged:
            response.success = False
            response.message = "Target already engaged"
            rospy.logwarn("Engagement request denied: Target already engaged")
            return response
        
        if not self.engagement_ready:
            response.success = False
            response.message = "Engagement conditions not met"
            rospy.logwarn("Engagement request denied: Conditions not met")
            return response
        
        # Execute engagement
        rospy.loginfo("="*60)
        rospy.loginfo("ENGAGING TARGET!")
        rospy.loginfo("="*60)
        
        # Simulate weapon fire
        self.fire_weapon()
        
        # Mark as engaged
        self.target_engaged = True
        self.engagement_complete = True
        
        # Calculate engagement metrics
        if self.uav_state and self.target_state:
            altitude = self.uav_state.pose.position.z
            distance = self.uav_state.distance_to_target
            lock_time = (rospy.Time.now() - self.lock_start_time).to_sec() if self.lock_start_time else 0
            
            response.success = True
            response.message = f"Target engaged successfully at altitude {altitude:.2f}m, " \
                             f"distance {distance:.2f}m, lock time {lock_time:.1f}s"
            
            rospy.loginfo(response.message)
            rospy.loginfo(f"Target ID: {self.target_state.track_id}")
            rospy.loginfo(f"Confidence: {self.target_state.confidence:.2f}")
            rospy.loginfo("="*60)
        else:
            response.success = True
            response.message = "Target engaged successfully"
        
        self.publish_status("ENGAGED: Target hit!")
        
        return response
    
    def fire_weapon(self):
        """Simulate weapon fire"""
        # Publish weapon fire event
        fire_msg = Bool()
        fire_msg.data = True
        self.weapon_fire_pub.publish(fire_msg)
        
        rospy.loginfo("🎯 WEAPON FIRED!")
        rospy.loginfo("💥 TARGET HIT!")
        
        # Could add visual/audio effects here in a real system
        # For simulation, we just log the event
    
    def publish_status(self, status):
        """Publish engagement status"""
        msg = String()
        msg.data = status
        self.engagement_status_pub.publish(msg)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Publish engagement ready status
            ready_msg = Bool()
            ready_msg.data = self.engagement_ready
            self.engagement_ready_pub.publish(ready_msg)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = EngagementController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

