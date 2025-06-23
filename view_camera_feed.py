#!/usr/bin/env python3
"""
ROS2 Camera Feed Viewer
Visualizes the rotating camera output from Isaac Sim
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers for RGB and Depth
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb',
            self.rgb_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10
        )
        
        self.get_logger().info('Camera Viewer started. Press ESC to quit.')
        
        # Image storage
        self.latest_rgb = None
        self.latest_depth = None
        
    def rgb_callback(self, msg):
        """Process RGB image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb = cv_image
            self.display_images()
            
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
            
    def depth_callback(self, msg):
        """Process depth image"""
        try:
            # Convert ROS depth image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            
            # Normalize depth for visualization (0-255)
            if cv_image.dtype == np.float32:
                # Convert from meters to millimeters and normalize
                depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = depth_normalized.astype(np.uint8)
            else:
                depth_normalized = cv_image
                
            # Apply colormap for better visualization
            self.latest_depth = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            self.display_images()
            
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
            
    def display_images(self):
        """Display RGB and depth images side by side"""
        if self.latest_rgb is not None and self.latest_depth is not None:
            # Resize images to same height
            h1, w1 = self.latest_rgb.shape[:2]
            h2, w2 = self.latest_depth.shape[:2]
            
            target_height = min(h1, h2, 400)  # Max height 400px
            
            # Resize RGB
            rgb_resized = cv2.resize(self.latest_rgb, 
                                   (int(w1 * target_height / h1), target_height))
            
            # Resize depth
            depth_resized = cv2.resize(self.latest_depth, 
                                     (int(w2 * target_height / h2), target_height))
            
            # Combine images horizontally
            combined = np.hstack([rgb_resized, depth_resized])
            
            # Add labels
            cv2.putText(combined, "RGB Camera (Rotating)", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined, "Depth Camera", (rgb_resized.shape[1] + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display
            cv2.imshow('Isaac Sim Rotating Camera Feed', combined)
            
            # Check for exit key
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.get_logger().info('Exit requested')
                rclpy.shutdown()

def main():
    rclpy.init()
    
    try:
        viewer = CameraViewer()
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
