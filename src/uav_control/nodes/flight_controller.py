#!/usr/bin/env python3
"""
Flight Controller Node
Implements autonomous flight behaviors:
- Takeoff to 10m altitude
- Search pattern at constant altitude
- Target approach with visual servoing
- Descent and engagement
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from uav_msgs.msg import UAVState, TargetState
from std_msgs.msg import Empty

class FlightController:
    # Flight modes
    MODE_IDLE = "IDLE"
    MODE_TAKEOFF = "TAKEOFF"
    MODE_SEARCH = "SEARCH"
    MODE_TRACKING = "TRACKING"
    MODE_APPROACH = "APPROACH"
    MODE_ENGAGE = "ENGAGE"
    MODE_COMPLETE = "MISSION_COMPLETE"
    
    def __init__(self):
        rospy.init_node('flight_controller', anonymous=False)
        
        # Parameters
        self.takeoff_altitude = rospy.get_param('~takeoff_altitude', 10.0)
        self.search_altitude = rospy.get_param('~search_altitude', 10.0)
        self.search_speed = rospy.get_param('~search_speed', 2.0)
        self.approach_speed = rospy.get_param('~approach_speed', 1.5)
        self.engagement_altitude = rospy.get_param('~engagement_altitude', 2.0)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.5)
        
        # State
        self.current_mode = self.MODE_IDLE
        self.current_pose = Pose()
        self.current_velocity = Twist()
        self.target_state = None
        self.target_locked = False
        
        # Search pattern state
        self.search_waypoints = []
        self.current_waypoint_idx = 0
        self.search_pattern_complete = False
        
        # Mission timing
        self.mission_start_time = None
        self.takeoff_start_time = None
        self.search_start_time = None
        self.tracking_start_time = None
        self.approach_start_time = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.uav_state_pub = rospy.Publisher('/uav/state', UAVState, queue_size=10)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, 
                                         self.odom_callback, queue_size=10)
        self.target_sub = rospy.Subscriber('/vision/target_state', TargetState,
                                          self.target_callback, queue_size=10)
        
        # Control loop timer
        self.control_rate = rospy.Rate(20)  # 20 Hz
        
        rospy.loginfo("Flight Controller initialized")
        
        # Start mission after brief delay
        rospy.Timer(rospy.Duration(2.0), self.start_mission, oneshot=True)
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist
    
    def target_callback(self, msg):
        """Update target state from vision system"""
        self.target_state = msg
        self.target_locked = msg.is_locked
    
    def start_mission(self, event):
        """Start the mission"""
        rospy.loginfo("Starting mission...")
        self.mission_start_time = rospy.Time.now()
        self.current_mode = self.MODE_TAKEOFF
        self.takeoff_start_time = rospy.Time.now()
    
    def generate_search_pattern(self):
        """Generate expanding square search pattern"""
        # Start from current position
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        z = self.search_altitude
        
        # Expanding square spiral
        waypoints = []
        side_length = 10.0
        num_loops = 3
        
        x, y = start_x, start_y
        
        for loop in range(num_loops):
            # Right
            x += side_length
            waypoints.append((x, y, z))
            
            # Forward
            y += side_length
            waypoints.append((x, y, z))
            
            # Left
            x -= side_length * 2
            waypoints.append((x, y, z))
            
            # Back
            y -= side_length * 2
            waypoints.append((x, y, z))
            
            # Return to start side
            x += side_length
            waypoints.append((x, y, z))
            
            side_length += 10.0
        
        self.search_waypoints = waypoints
        self.current_waypoint_idx = 0
        
        rospy.loginfo(f"Generated search pattern with {len(waypoints)} waypoints")
    
    def control_loop(self):
        """Main control loop"""
        while not rospy.is_shutdown():
            # Execute behavior based on current mode
            if self.current_mode == self.MODE_TAKEOFF:
                self.execute_takeoff()
            elif self.current_mode == self.MODE_SEARCH:
                self.execute_search()
            elif self.current_mode == self.MODE_TRACKING:
                self.execute_tracking()
            elif self.current_mode == self.MODE_APPROACH:
                self.execute_approach()
            elif self.current_mode == self.MODE_ENGAGE:
                self.execute_engagement()
            elif self.current_mode == self.MODE_COMPLETE:
                self.execute_mission_complete()
            
            # Publish UAV state
            self.publish_uav_state()
            
            self.control_rate.sleep()
    
    def execute_takeoff(self):
        """Execute takeoff to target altitude"""
        current_alt = self.current_pose.position.z
        
        if current_alt < self.takeoff_altitude - self.position_tolerance:
            # Ascend
            cmd = Twist()
            cmd.linear.z = 1.0  # Upward velocity
            self.cmd_vel_pub.publish(cmd)
        else:
            # Takeoff complete
            rospy.loginfo(f"Takeoff complete at {current_alt:.2f}m")
            self.current_mode = self.MODE_SEARCH
            self.search_start_time = rospy.Time.now()
            self.generate_search_pattern()
    
    def execute_search(self):
        """Execute search pattern"""
        # Check if target detected
        if self.target_locked:
            rospy.loginfo("Target detected! Switching to tracking mode")
            self.current_mode = self.MODE_TRACKING
            self.tracking_start_time = rospy.Time.now()
            return
        
        # Navigate to next waypoint
        if self.current_waypoint_idx < len(self.search_waypoints):
            target_x, target_y, target_z = self.search_waypoints[self.current_waypoint_idx]
            
            # Compute error
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            dz = target_z - self.current_pose.position.z
            
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.position_tolerance:
                # Reached waypoint, move to next
                self.current_waypoint_idx += 1
                rospy.loginfo(f"Reached waypoint {self.current_waypoint_idx}/{len(self.search_waypoints)}")
            else:
                # Move toward waypoint
                cmd = Twist()
                cmd.linear.x = self.search_speed * dx / distance
                cmd.linear.y = self.search_speed * dy / distance
                cmd.linear.z = 0.5 * dz  # Maintain altitude
                self.cmd_vel_pub.publish(cmd)
        else:
            # Search pattern complete, hover and wait
            if not self.search_pattern_complete:
                rospy.logwarn("Search pattern complete, no target found. Hovering...")
                self.search_pattern_complete = True
            
            cmd = Twist()  # Hover
            self.cmd_vel_pub.publish(cmd)
    
    def execute_tracking(self):
        """Track target while maintaining altitude"""
        if not self.target_locked:
            rospy.logwarn("Target lost during tracking")
            # Could return to search or hover
            return
        
        # Visual servoing: keep target centered in image
        # Image center is approximately (320, 240) for 640x480 image
        image_center_x = 320
        image_center_y = 240
        
        # Target center in image
        target_center_x = self.target_state.bbox_x + self.target_state.bbox_width / 2
        target_center_y = self.target_state.bbox_y + self.target_state.bbox_height / 2
        
        # Error in pixels
        error_x = target_center_x - image_center_x
        error_y = target_center_y - image_center_y
        
        # Simple proportional control
        # Positive error_x means target is to the right, move right
        # Positive error_y means target is below center, move forward (camera is downward-facing)
        kp_xy = 0.002  # Proportional gain
        
        cmd = Twist()
        cmd.linear.x = kp_xy * error_y  # Forward/backward
        cmd.linear.y = kp_xy * error_x  # Left/right
        cmd.linear.z = 0.0  # Maintain altitude
        
        # Limit velocities
        max_vel = 1.0
        cmd.linear.x = np.clip(cmd.linear.x, -max_vel, max_vel)
        cmd.linear.y = np.clip(cmd.linear.y, -max_vel, max_vel)
        
        self.cmd_vel_pub.publish(cmd)
        
        # Check if target is well-centered and we can start approach
        if abs(error_x) < 50 and abs(error_y) < 50:
            # Target centered, start approach
            rospy.loginfo("Target centered, starting approach")
            self.current_mode = self.MODE_APPROACH
            self.approach_start_time = rospy.Time.now()
    
    def execute_approach(self):
        """Approach target with descent"""
        if not self.target_locked:
            rospy.logwarn("Target lost during approach")
            self.current_mode = self.MODE_TRACKING
            return
        
        current_alt = self.current_pose.position.z
        
        # Visual servoing to keep target centered
        image_center_x = 320
        image_center_y = 240
        
        target_center_x = self.target_state.bbox_x + self.target_state.bbox_width / 2
        target_center_y = self.target_state.bbox_y + self.target_state.bbox_height / 2
        
        error_x = target_center_x - image_center_x
        error_y = target_center_y - image_center_y
        
        kp_xy = 0.002
        
        cmd = Twist()
        cmd.linear.x = kp_xy * error_y
        cmd.linear.y = kp_xy * error_x
        
        # Descend toward target
        if current_alt > self.engagement_altitude:
            cmd.linear.z = -0.5  # Descend
        else:
            cmd.linear.z = 0.0
        
        # Limit velocities
        max_vel = 1.0
        cmd.linear.x = np.clip(cmd.linear.x, -max_vel, max_vel)
        cmd.linear.y = np.clip(cmd.linear.y, -max_vel, max_vel)
        
        self.cmd_vel_pub.publish(cmd)
        
        # Check if reached engagement altitude
        if current_alt <= self.engagement_altitude + self.position_tolerance:
            rospy.loginfo("Reached engagement altitude")
            self.current_mode = self.MODE_ENGAGE
    
    def execute_engagement(self):
        """Execute target engagement"""
        rospy.loginfo("ENGAGING TARGET!")
        
        # Hover over target
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # In a real system, this would trigger weapon release, etc.
        # For simulation, we just mark mission complete
        rospy.sleep(2.0)
        
        rospy.loginfo("Target engaged successfully!")
        self.current_mode = self.MODE_COMPLETE
    
    def execute_mission_complete(self):
        """Mission complete - hover in place"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Log mission statistics once
        if not hasattr(self, 'mission_logged'):
            self.log_mission_statistics()
            self.mission_logged = True
    
    def publish_uav_state(self):
        """Publish current UAV state"""
        msg = UAVState()
        msg.header.stamp = rospy.Time.now()
        msg.flight_mode = self.current_mode
        msg.pose = self.current_pose
        msg.velocity = self.current_velocity
        
        if self.target_state is not None:
            msg.target_detected = self.target_locked
            msg.target_track_id = self.target_state.track_id
            
            # Estimate distance to target (simplified)
            # In real system, would use camera calibration and target size
            msg.distance_to_target = self.current_pose.position.z
        else:
            msg.target_detected = False
            msg.target_track_id = -1
            msg.distance_to_target = -1.0
        
        # Mission progress
        if self.current_mode == self.MODE_IDLE:
            msg.mission_progress = 0.0
        elif self.current_mode == self.MODE_TAKEOFF:
            msg.mission_progress = 0.1
        elif self.current_mode == self.MODE_SEARCH:
            msg.mission_progress = 0.3
        elif self.current_mode == self.MODE_TRACKING:
            msg.mission_progress = 0.5
        elif self.current_mode == self.MODE_APPROACH:
            msg.mission_progress = 0.8
        elif self.current_mode == self.MODE_ENGAGE:
            msg.mission_progress = 0.95
        else:
            msg.mission_progress = 1.0
        
        self.uav_state_pub.publish(msg)
    
    def log_mission_statistics(self):
        """Log mission statistics"""
        if self.mission_start_time is None:
            return
        
        total_time = (rospy.Time.now() - self.mission_start_time).to_sec()
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("MISSION COMPLETE")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"Total mission time: {total_time:.2f} seconds")
        
        if self.takeoff_start_time:
            takeoff_time = (self.search_start_time - self.takeoff_start_time).to_sec() if self.search_start_time else 0
            rospy.loginfo(f"Takeoff time: {takeoff_time:.2f} seconds")
        
        if self.search_start_time and self.tracking_start_time:
            search_time = (self.tracking_start_time - self.search_start_time).to_sec()
            rospy.loginfo(f"Search time: {search_time:.2f} seconds")
        
        if self.tracking_start_time and self.approach_start_time:
            tracking_time = (self.approach_start_time - self.tracking_start_time).to_sec()
            rospy.loginfo(f"Tracking time: {tracking_time:.2f} seconds")
        
        rospy.loginfo("=" * 50)
    
    def run(self):
        """Main run loop"""
        self.control_loop()

if __name__ == '__main__':
    try:
        controller = FlightController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

