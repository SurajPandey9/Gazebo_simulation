#!/usr/bin/env python3
"""
Object Detector Node
Uses YOLOv5 for real-time object detection of tanks from UAV camera feed.
"""

import rospy
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from uav_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import Point

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=False)
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.nms_threshold = rospy.get_param('~nms_threshold', 0.4)
        self.model_size = rospy.get_param('~model_size', 'yolov5s')  # yolov5s, yolov5m, yolov5l
        self.target_class = rospy.get_param('~target_class', 'tank')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLOv5 model
        rospy.loginfo("Loading YOLOv5 model...")
        try:
            # Use torch hub to load YOLOv5
            self.model = torch.hub.load('ultralytics/yolov5', self.model_size, pretrained=True)
            self.model.conf = self.confidence_threshold
            self.model.iou = self.nms_threshold
            
            # For our simulation, we'll use 'truck' class as proxy for tank
            # In real scenario, you'd train custom model with tank class
            self.detection_classes = ['truck', 'car', 'bus']  # Vehicle classes
            
            rospy.loginfo("YOLOv5 model loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv5 model: {e}")
            rospy.logwarn("Falling back to simple color-based detection")
            self.model = None
        
        # Publishers
        self.detection_pub = rospy.Publisher('/vision/detections', DetectionArray, queue_size=10)
        self.debug_image_pub = rospy.Publisher('/vision/debug_image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/uav/camera/image_raw', Image, self.image_callback, queue_size=1)
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        
        rospy.loginfo("Object Detector initialized")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform detection
            detections = self.detect_objects(cv_image)
            
            # Publish detections
            detection_array = DetectionArray()
            detection_array.header = msg.header
            detection_array.detections = detections
            self.detection_pub.publish(detection_array)
            
            # Create and publish debug image
            debug_image = self.draw_detections(cv_image, detections)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
            # Update statistics
            self.frame_count += 1
            if len(detections) > 0:
                self.detection_count += 1
            
            if self.frame_count % 30 == 0:
                rospy.loginfo(f"Processed {self.frame_count} frames, {self.detection_count} with detections")
                
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
    
    def detect_objects(self, image):
        """Detect objects in image using YOLOv5 or fallback method"""
        detections = []
        
        if self.model is not None:
            # YOLOv5 detection
            detections = self.yolo_detect(image)
        else:
            # Fallback: Simple color-based detection for dark objects (tank)
            detections = self.color_based_detect(image)
        
        return detections
    
    def yolo_detect(self, image):
        """Perform YOLOv5 detection"""
        detections = []
        
        # Run inference
        results = self.model(image)
        
        # Process results
        for *box, conf, cls in results.xyxy[0]:
            class_name = self.model.names[int(cls)]
            
            # Filter for vehicle classes (proxy for tank)
            if class_name in self.detection_classes:
                detection = Detection()
                detection.x = int(box[0])
                detection.y = int(box[1])
                detection.width = int(box[2] - box[0])
                detection.height = int(box[3] - box[1])
                detection.class_name = 'tank'  # Map to our target class
                detection.confidence = float(conf)
                detection.track_id = -1  # Will be assigned by tracker
                detection.has_3d_position = False
                
                detections.append(detection)
        
        return detections
    
    def color_based_detect(self, image):
        """
        Fallback detection method using color and size filtering.
        Detects dark, large objects that could be tanks.
        """
        detections = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Threshold for dark objects
        _, binary = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        
        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter contours by size
        min_area = 500  # Minimum area for tank detection
        max_area = 50000  # Maximum area
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if min_area < area < max_area:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Aspect ratio check (tanks are roughly rectangular)
                aspect_ratio = float(w) / h if h > 0 else 0
                if 0.5 < aspect_ratio < 3.0:
                    detection = Detection()
                    detection.x = x
                    detection.y = y
                    detection.width = w
                    detection.height = h
                    detection.class_name = 'tank'
                    detection.confidence = 0.7  # Fixed confidence for color-based
                    detection.track_id = -1
                    detection.has_3d_position = False
                    
                    detections.append(detection)
        
        return detections
    
    def draw_detections(self, image, detections):
        """Draw bounding boxes on image for visualization"""
        debug_image = image.copy()
        
        for det in detections:
            # Draw bounding box
            color = (0, 255, 0)  # Green for detections
            cv2.rectangle(debug_image, 
                         (det.x, det.y), 
                         (det.x + det.width, det.y + det.height),
                         color, 2)
            
            # Draw label
            label = f"{det.class_name}: {det.confidence:.2f}"
            cv2.putText(debug_image, label, 
                       (det.x, det.y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Add frame info
        info_text = f"Detections: {len(detections)}"
        cv2.putText(debug_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return debug_image
    
    def run(self):
        """Main loop"""
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass

