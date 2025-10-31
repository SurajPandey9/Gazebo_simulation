#!/usr/bin/env python3
"""
Object Tracker Node
Implements DeepSORT-inspired tracking with re-identification capabilities.
Maintains target identity across frames, handles occlusions and re-acquisitions.
"""

import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from uav_msgs.msg import Detection, DetectionArray, TargetState
from geometry_msgs.msg import Point, Pose, Twist
from filterpy.kalman import KalmanFilter

class Track:
    """Represents a single tracked object"""
    
    next_id = 1
    
    def __init__(self, detection, feature_vector=None):
        self.track_id = Track.next_id
        Track.next_id += 1
        
        # State
        self.bbox = [detection.x, detection.y, detection.width, detection.height]
        self.class_name = detection.class_name
        self.confidence = detection.confidence
        
        # Tracking state
        self.frames_tracked = 1
        self.frames_lost = 0
        self.is_confirmed = False
        self.is_deleted = False
        self.is_reidentified = False
        
        # Feature for re-identification
        self.feature_vector = feature_vector
        self.feature_history = deque(maxlen=30)
        if feature_vector is not None:
            self.feature_history.append(feature_vector)
        
        # Kalman filter for motion prediction
        self.kf = self._init_kalman_filter()
        
        # History
        self.position_history = deque(maxlen=30)
        self.position_history.append(self._get_center())
    
    def _init_kalman_filter(self):
        """Initialize Kalman filter for tracking"""
        kf = KalmanFilter(dim_x=8, dim_z=4)
        
        # State: [x, y, w, h, vx, vy, vw, vh]
        # Measurement: [x, y, w, h]
        
        # State transition matrix
        dt = 1.0
        kf.F = np.array([
            [1, 0, 0, 0, dt, 0, 0, 0],
            [0, 1, 0, 0, 0, dt, 0, 0],
            [0, 0, 1, 0, 0, 0, dt, 0],
            [0, 0, 0, 1, 0, 0, 0, dt],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix
        kf.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ])
        
        # Initialize state
        kf.x[:4] = np.array(self.bbox)
        
        # Covariance matrices
        kf.P *= 10.0
        kf.R *= 1.0
        kf.Q[-4:, -4:] *= 0.01
        
        return kf
    
    def predict(self):
        """Predict next state using Kalman filter"""
        self.kf.predict()
        predicted_bbox = self.kf.x[:4].copy()
        return predicted_bbox
    
    def update(self, detection, feature_vector=None):
        """Update track with new detection"""
        measurement = np.array([detection.x, detection.y, detection.width, detection.height])
        self.kf.update(measurement)
        
        self.bbox = self.kf.x[:4].copy()
        self.confidence = detection.confidence
        self.frames_tracked += 1
        self.frames_lost = 0
        
        # Update feature history
        if feature_vector is not None:
            self.feature_history.append(feature_vector)
        
        # Update position history
        self.position_history.append(self._get_center())
        
        # Confirm track after minimum frames
        if self.frames_tracked >= 3:
            self.is_confirmed = True
    
    def mark_missed(self):
        """Mark track as missed in current frame"""
        self.frames_lost += 1
        
        # Delete track if lost for too long
        if self.frames_lost > 30:
            self.is_deleted = True
    
    def _get_center(self):
        """Get center point of bounding box"""
        return (self.bbox[0] + self.bbox[2] / 2, self.bbox[1] + self.bbox[3] / 2)
    
    def get_velocity(self):
        """Estimate velocity from Kalman filter"""
        return (self.kf.x[4], self.kf.x[5])


class ObjectTracker:
    def __init__(self):
        rospy.init_node('object_tracker', anonymous=False)
        
        # Parameters
        self.max_iou_distance = rospy.get_param('~max_iou_distance', 0.7)
        self.max_feature_distance = rospy.get_param('~max_feature_distance', 0.3)
        self.min_confidence = rospy.get_param('~min_confidence', 0.5)
        
        # Tracking state
        self.tracks = []
        self.frame_count = 0
        self.reidentification_count = 0
        
        # CV Bridge
        self.bridge = CvBridge()
        self.current_image = None
        
        # Publishers
        self.target_pub = rospy.Publisher('/vision/target_state', TargetState, queue_size=10)
        self.tracked_image_pub = rospy.Publisher('/vision/tracked_image', Image, queue_size=1)
        
        # Subscribers
        self.detection_sub = rospy.Subscriber('/vision/detections', DetectionArray, 
                                              self.detection_callback, queue_size=10)
        self.image_sub = rospy.Subscriber('/uav/camera/image_raw', Image, 
                                         self.image_callback, queue_size=1)
        
        rospy.loginfo("Object Tracker initialized")
    
    def image_callback(self, msg):
        """Store current image for feature extraction"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def detection_callback(self, msg):
        """Process detections and update tracks"""
        self.frame_count += 1
        
        # Predict next state for all tracks
        for track in self.tracks:
            track.predict()
        
        # Extract features for detections
        detection_features = []
        for det in msg.detections:
            if self.current_image is not None:
                feature = self.extract_features(self.current_image, det)
                detection_features.append(feature)
            else:
                detection_features.append(None)
        
        # Associate detections with tracks
        matches, unmatched_detections, unmatched_tracks = self.associate_detections_to_tracks(
            msg.detections, detection_features)
        
        # Update matched tracks
        for track_idx, det_idx in matches:
            self.tracks[track_idx].update(msg.detections[det_idx], detection_features[det_idx])
        
        # Mark unmatched tracks as missed
        for track_idx in unmatched_tracks:
            self.tracks[track_idx].mark_missed()
        
        # Try to re-identify lost tracks
        if len(unmatched_detections) > 0 and len(unmatched_tracks) > 0:
            reid_matches = self.reidentify_tracks(msg.detections, detection_features,
                                                  unmatched_detections, unmatched_tracks)
            for track_idx, det_idx in reid_matches:
                self.tracks[track_idx].update(msg.detections[det_idx], detection_features[det_idx])
                self.tracks[track_idx].is_reidentified = True
                self.reidentification_count += 1
                rospy.loginfo(f"Re-identified track {self.tracks[track_idx].track_id}")
                unmatched_detections.remove(det_idx)
        
        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            if msg.detections[det_idx].confidence >= self.min_confidence:
                new_track = Track(msg.detections[det_idx], detection_features[det_idx])
                self.tracks.append(new_track)
                rospy.loginfo(f"Created new track {new_track.track_id}")
        
        # Remove deleted tracks
        self.tracks = [t for t in self.tracks if not t.is_deleted]
        
        # Publish target state for primary target (highest confidence confirmed track)
        self.publish_target_state(msg.header)
        
        # Publish visualization
        if self.current_image is not None:
            self.publish_visualization(msg.header)
    
    def associate_detections_to_tracks(self, detections, features):
        """Associate detections to existing tracks using IoU and features"""
        if len(self.tracks) == 0:
            return [], list(range(len(detections))), []
        
        if len(detections) == 0:
            return [], [], list(range(len(self.tracks)))
        
        # Compute cost matrix (IoU distance)
        cost_matrix = np.zeros((len(self.tracks), len(detections)))
        
        for t, track in enumerate(self.tracks):
            for d, det in enumerate(detections):
                det_bbox = [det.x, det.y, det.width, det.height]
                iou = self.compute_iou(track.bbox, det_bbox)
                cost_matrix[t, d] = 1 - iou  # Convert IoU to distance
        
        # Simple greedy matching (in production, use Hungarian algorithm)
        matches = []
        unmatched_tracks = list(range(len(self.tracks)))
        unmatched_detections = list(range(len(detections)))
        
        while len(unmatched_tracks) > 0 and len(unmatched_detections) > 0:
            # Find minimum cost
            min_cost = float('inf')
            min_track = -1
            min_det = -1
            
            for t in unmatched_tracks:
                for d in unmatched_detections:
                    if cost_matrix[t, d] < min_cost:
                        min_cost = cost_matrix[t, d]
                        min_track = t
                        min_det = d
            
            # Check if match is valid
            if min_cost < self.max_iou_distance:
                matches.append((min_track, min_det))
                unmatched_tracks.remove(min_track)
                unmatched_detections.remove(min_det)
            else:
                break
        
        return matches, unmatched_detections, unmatched_tracks
    
    def reidentify_tracks(self, detections, features, unmatched_det_indices, unmatched_track_indices):
        """Attempt to re-identify lost tracks using appearance features"""
        reid_matches = []
        
        for track_idx in unmatched_track_indices:
            track = self.tracks[track_idx]
            
            # Only try to re-identify recently lost confirmed tracks
            if not track.is_confirmed or track.frames_lost > 10:
                continue
            
            # Find best matching detection based on features
            best_match = -1
            best_distance = float('inf')
            
            for det_idx in unmatched_det_indices:
                if features[det_idx] is None or len(track.feature_history) == 0:
                    continue
                
                # Compute feature distance
                distance = self.compute_feature_distance(track.feature_history, features[det_idx])
                
                if distance < best_distance and distance < self.max_feature_distance:
                    best_distance = distance
                    best_match = det_idx
            
            if best_match != -1:
                reid_matches.append((track_idx, best_match))
        
        return reid_matches
    
    @staticmethod
    def compute_iou(bbox1, bbox2):
        """Compute Intersection over Union"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # Compute intersection
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        intersection = (x_right - x_left) * (y_bottom - y_top)
        
        # Compute union
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def extract_features(self, image, detection):
        """Extract appearance features from detection region"""
        try:
            # Crop detection region
            x, y, w, h = detection.x, detection.y, detection.width, detection.height
            x, y = max(0, x), max(0, y)
            crop = image[y:y+h, x:x+w]
            
            if crop.size == 0:
                return None
            
            # Resize to fixed size
            crop_resized = cv2.resize(crop, (64, 64))
            
            # Compute color histogram as simple feature
            hist_b = cv2.calcHist([crop_resized], [0], None, [32], [0, 256])
            hist_g = cv2.calcHist([crop_resized], [1], None, [32], [0, 256])
            hist_r = cv2.calcHist([crop_resized], [2], None, [32], [0, 256])
            
            # Concatenate and normalize
            feature = np.concatenate([hist_b, hist_g, hist_r]).flatten()
            feature = feature / (np.linalg.norm(feature) + 1e-6)
            
            return feature
            
        except Exception as e:
            rospy.logwarn(f"Feature extraction failed: {e}")
            return None
    
    @staticmethod
    def compute_feature_distance(feature_history, feature):
        """Compute distance between feature and feature history"""
        # Use minimum distance to any historical feature
        distances = []
        for hist_feature in feature_history:
            # Cosine distance
            distance = 1 - np.dot(hist_feature, feature)
            distances.append(distance)
        
        return min(distances) if distances else float('inf')
    
    def publish_target_state(self, header):
        """Publish state of primary target"""
        # Find best confirmed track
        confirmed_tracks = [t for t in self.tracks if t.is_confirmed]
        
        if len(confirmed_tracks) == 0:
            return
        
        # Select track with highest confidence and most frames tracked
        primary_track = max(confirmed_tracks, key=lambda t: (t.confidence, t.frames_tracked))
        
        # Create message
        msg = TargetState()
        msg.header = header
        msg.track_id = primary_track.track_id
        msg.class_name = primary_track.class_name
        msg.is_locked = True
        msg.confidence = primary_track.confidence
        msg.frames_tracked = primary_track.frames_tracked
        msg.frames_lost = primary_track.frames_lost
        msg.is_reidentified = primary_track.is_reidentified
        msg.bbox_x = int(primary_track.bbox[0])
        msg.bbox_y = int(primary_track.bbox[1])
        msg.bbox_width = int(primary_track.bbox[2])
        msg.bbox_height = int(primary_track.bbox[3])
        
        # Velocity estimate
        msg.velocity.linear.x, msg.velocity.linear.y = primary_track.get_velocity()
        
        self.target_pub.publish(msg)
    
    def publish_visualization(self, header):
        """Publish image with tracking visualization"""
        if self.current_image is None:
            return
        
        vis_image = self.current_image.copy()
        
        for track in self.tracks:
            if not track.is_confirmed:
                continue
            
            x, y, w, h = [int(v) for v in track.bbox]
            
            # Color based on state
            if track.frames_lost > 0:
                color = (0, 165, 255)  # Orange for lost
            elif track.is_reidentified:
                color = (255, 0, 255)  # Magenta for re-identified
            else:
                color = (0, 255, 0)  # Green for active
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            label = f"ID:{track.track_id} ({track.frames_tracked}f)"
            cv2.putText(vis_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw trajectory
            if len(track.position_history) > 1:
                points = np.array(track.position_history, dtype=np.int32)
                cv2.polylines(vis_image, [points], False, color, 2)
        
        # Add statistics
        stats = f"Tracks: {len(self.tracks)} | ReID: {self.reidentification_count}"
        cv2.putText(vis_image, stats, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Publish
        try:
            msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            msg.header = header
            self.tracked_image_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error publishing visualization: {e}")
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = ObjectTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass

