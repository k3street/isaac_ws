#!/usr/bin/env python3
"""
Isaac Sim Camera Subscriber Node
Subscribes to camera data from Isaac Sim and processes RGB/Depth images
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class IsaacCameraSubscriber(Node):
    def __init__(self):
        super().__init__('isaac_camera_subscriber')
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Create subscribers for all Isaac Sim camera topics
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/rgb',
            self.rgb_callback,
            10
        )
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10
        )
        
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Create publisher for analysis results
        self.analysis_publisher = self.create_publisher(
            String,
            '/camera/analysis',
            10
        )
        
        # Create a status publisher
        self.status_publisher = self.create_publisher(
            String,
            '/isaac_test/status',
            10
        )
        
        # Create timer for status updates
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_camera_info = None
        
        # Statistics
        self.rgb_count = 0
        self.depth_count = 0
        self.camera_info_count = 0
        self.start_time = time.time()
        
        # Display window settings
        self.show_gui = True  # Set to False to disable OpenCV windows
        
        self.get_logger().info('🚀 Isaac Camera Subscriber Node started!')
        self.get_logger().info('📹 Subscribing to Isaac Sim camera topics:')
        self.get_logger().info('   - /camera/rgb')
        self.get_logger().info('   - /camera/depth')
        self.get_logger().info('   - /camera/camera_info')
        
    def rgb_callback(self, msg):
        """Process RGB image from Isaac Sim camera"""
        try:
            self.rgb_count += 1
            
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb = cv_image
            
            # Perform some basic image analysis
            height, width = cv_image.shape[:2]
            
            # Calculate image statistics
            mean_brightness = np.mean(cv_image)
            
            # Add text overlay with camera info
            display_image = cv_image.copy()
            cv2.putText(display_image, f"RGB Frame #{self.rgb_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_image, f"Size: {width}x{height}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(display_image, f"Brightness: {mean_brightness:.1f}", (10, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(display_image, "Isaac Sim Rotating Camera", (10, height-20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Display the image
            if self.show_gui:
                cv2.imshow('Isaac Sim RGB Camera', display_image)
                cv2.waitKey(1)
                
            # Publish analysis results
            analysis_msg = String()
            analysis_msg.data = f"RGB analysis: brightness={mean_brightness:.1f}, size={width}x{height}"
            self.analysis_publisher.publish(analysis_msg)
            
            # Log periodically
            if self.rgb_count % 30 == 0:  # Every 30 frames
                self.get_logger().info(f'📷 RGB: {self.rgb_count} frames received, brightness: {mean_brightness:.1f}')
                
        except Exception as e:
            self.get_logger().error(f'❌ RGB processing error: {e}')
            
    def depth_callback(self, msg):
        """Process depth image from Isaac Sim camera"""
        try:
            self.depth_count += 1
            
            # Convert ROS depth image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.latest_depth = cv_image
            
            # Process depth data
            if cv_image.dtype == np.float32:
                # Convert meters to millimeters for better visualization
                depth_mm = cv_image * 1000.0
                
                # Calculate depth statistics
                valid_pixels = depth_mm[depth_mm > 0]
                if len(valid_pixels) > 0:
                    min_depth = np.min(valid_pixels)
                    max_depth = np.max(valid_pixels)
                    mean_depth = np.mean(valid_pixels)
                else:
                    min_depth = max_depth = mean_depth = 0
                
                # Normalize for visualization (0-255)
                depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_normalized = depth_normalized.astype(np.uint8)
                
                # Apply colormap for better visualization
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                
                # Add text overlay
                cv2.putText(depth_colored, f"Depth Frame #{self.depth_count}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(depth_colored, f"Range: {min_depth:.1f}-{max_depth:.1f}mm", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(depth_colored, f"Mean: {mean_depth:.1f}mm", (10, 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Display the depth image
                if self.show_gui:
                    cv2.imshow('Isaac Sim Depth Camera', depth_colored)
                    cv2.waitKey(1)
                    
                # Log periodically
                if self.depth_count % 30 == 0:
                    self.get_logger().info(f'🌊 Depth: {self.depth_count} frames, range: {min_depth:.1f}-{max_depth:.1f}mm')
            
        except Exception as e:
            self.get_logger().error(f'❌ Depth processing error: {e}')
            
    def camera_info_callback(self, msg):
        """Process camera info from Isaac Sim"""
        try:
            self.camera_info_count += 1
            self.latest_camera_info = msg
            
            # Extract camera parameters
            width = msg.width
            height = msg.height
            fx = msg.k[0]  # Focal length X
            fy = msg.k[4]  # Focal length Y
            cx = msg.k[2]  # Principal point X
            cy = msg.k[5]  # Principal point Y
            
            # Log camera info periodically
            if self.camera_info_count % 60 == 0:  # Every 60 messages
                self.get_logger().info(f'📐 Camera Info: {width}x{height}, fx={fx:.1f}, fy={fy:.1f}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Camera info processing error: {e}')
            
    def publish_status(self):
        """Publish node status periodically"""
        elapsed = time.time() - self.start_time
        
        # Calculate rates
        rgb_rate = self.rgb_count / elapsed if elapsed > 0 else 0
        depth_rate = self.depth_count / elapsed if elapsed > 0 else 0
        
        status_msg = String()
        status_msg.data = f"Isaac Camera Subscriber Status: RGB={self.rgb_count} ({rgb_rate:.1f} Hz), Depth={self.depth_count} ({depth_rate:.1f} Hz), CameraInfo={self.camera_info_count}, Runtime={elapsed:.1f}s"
        
        self.status_publisher.publish(status_msg)
        
        # Log comprehensive status
        self.get_logger().info(f'📊 Status: RGB={self.rgb_count}@{rgb_rate:.1f}Hz, Depth={self.depth_count}@{depth_rate:.1f}Hz, Info={self.camera_info_count}, Up={elapsed:.1f}s')
        
    def create_combined_view(self):
        """Create a combined view of RGB and Depth images"""
        if self.latest_rgb is not None and self.latest_depth is not None:
            try:
                # Resize images to same height
                h1, w1 = self.latest_rgb.shape[:2]
                h2, w2 = self.latest_depth.shape[:2]
                
                target_height = min(h1, h2, 400)  # Max height 400px
                
                # Resize RGB
                rgb_resized = cv2.resize(self.latest_rgb, 
                                       (int(w1 * target_height / h1), target_height))
                
                # Process depth for display
                if self.latest_depth.dtype == np.float32:
                    depth_normalized = cv2.normalize(self.latest_depth, None, 0, 255, cv2.NORM_MINMAX)
                    depth_normalized = depth_normalized.astype(np.uint8)
                    depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                else:
                    depth_colored = self.latest_depth
                    
                # Resize depth
                depth_resized = cv2.resize(depth_colored, 
                                         (int(w2 * target_height / h2), target_height))
                
                # Combine horizontally
                combined = np.hstack([rgb_resized, depth_resized])
                
                # Add labels
                cv2.putText(combined, "RGB (Rotating Camera)", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined, "Depth", (rgb_resized.shape[1] + 10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                return combined
                
            except Exception as e:
                self.get_logger().error(f'❌ Combined view error: {e}')
                return None
                
        return None
        
    def destroy_node(self):
        """Clean up when node is destroyed"""
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create and spin the subscriber node
        subscriber = IsaacCameraSubscriber()
        
        print("🚀 Isaac Camera Subscriber running...")
        print("📹 Receiving camera data from Isaac Sim")
        print("🔄 Camera should be rotating every second")
        print("👁️  Watch OpenCV windows for live feed")
        print("📊 Check /isaac_test/status topic for statistics")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(subscriber)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutdown requested")
        
    finally:
        if 'subscriber' in locals():
            subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
