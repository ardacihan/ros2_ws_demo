#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from datetime import datetime
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImagePublisher(Node):
    
    def __init__(self):
        super().__init__('image_publisher')
        
        self.get_logger().info('Initializing image publisher...')
        
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)  # topic for raw images
        self.bridge = CvBridge()
        self.frame_count = 1000
        
        # Initialize camera (0 is the webcam)
        self.camera = cv2.VideoCapture(0)
        self.use_blank_image = False
        
        if not self.camera.isOpened():
            self.get_logger().warn('Failed to open camera, using blank image instead...')
            self.use_blank_image = True
            self.blank_image = self.create_blank_image(640, 480)
        else:
            self.get_logger().info('Camera opened successfully')
        
        # Timer 
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def create_blank_image(self, width, height):
        """Create a blank gray image with text"""
        blank = 128 * np.ones((height, width, 3), dtype=np.uint8)
        
        # Add text
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "NO CAMERA - BLANK IMAGE"
        text_size = cv2.getTextSize(text, font, 1, 2)[0]
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        
        cv2.putText(blank, text, (text_x, text_y), font, 1, (255, 255, 255), 2)
        
        return blank
    
    def timer_callback(self):
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info("No subscribers connected, not publishing any images....")
        else:
            if self.use_blank_image:
                frame = self.blank_image.copy()
                captured = True
            else:
                captured, frame = self.camera.read()
            
            if captured:
                timestamp = self.get_clock().now()
                time_sec = timestamp.nanoseconds / 1e9
                readable_time = datetime.fromtimestamp(time_sec).strftime("%H:%M:%S.%f")[:-3]
                
                # OpenCV to ROS2
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                
                self.get_logger().info(
                    f'Publishing frame {self.frame_count} at time: {readable_time}'
                )
                
                self.publisher_.publish(msg)
                self.frame_count += 1
            else:
                self.get_logger().warn('Failed to capture frame')
            
            self.get_logger().info("Message published successfully")
    
    def __del__(self):
        if hasattr(self, 'camera') and not self.use_blank_image:
            self.camera.release()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()