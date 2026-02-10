#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime
import cv2
import json
import os
import shutil


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        
        self.get_logger().info('Initializing image subscriber...')
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        self.frame_count = 1000
        
        self.output_root = 'output'
        self.images_dir = os.path.join(self.output_root, 'images')
        self.meta_dir = os.path.join(self.output_root, 'metafiles')

        # Remove previous output folder completely
        if os.path.exists(self.output_root):
            self.get_logger().info(f'Clearing existing directory: {self.output_root}')
            shutil.rmtree(self.output_root)

        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.meta_dir, exist_ok=True)

        self.get_logger().info('Image subscriber started.')
        self.get_logger().info(f'Saving images to: {os.path.abspath(self.images_dir)}')
        self.get_logger().info(f'Saving metadata to: {os.path.abspath(self.meta_dir)}')

    def listener_callback(self, msg):
        receive_time = self.get_clock().now()
        time_sec = receive_time.nanoseconds / 1e9
        readable_time = datetime.fromtimestamp(time_sec).strftime("%H:%M:%S.%f")[:-3]
        
        self.get_logger().info(f'Received frame {self.frame_count} at time: {readable_time}')
        
        # ROS to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        timestamp_text = f'Frame: {self.frame_count} | {readable_time}'
        
        # Add timestamp to image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 1
        text_color = (255, 0, 0)
        bg_color = (0, 0, 0)
        
        (text_width, text_height), baseline = cv2.getTextSize(
            timestamp_text, font, font_scale, font_thickness)
        
        cv2.rectangle(cv_image, (10, 10),
                      (20 + text_width, 20 + text_height + baseline),
                      bg_color, -1)
        
        cv2.putText(cv_image, timestamp_text, (15, 25 + text_height),
                    font, font_scale, text_color, font_thickness)
        
        image_filename = f'image_{self.frame_count:04d}.jpg'
        json_filename = f'image_{self.frame_count:04d}.json'
        
        image_path = os.path.join(self.images_dir, image_filename)
        json_path = os.path.join(self.meta_dir, json_filename)
        
        cv2.imwrite(image_path, cv_image)
        
        metadata = {
            'frame_number': self.frame_count,
            'image_filename': image_filename,
            'timestamp': datetime.fromtimestamp(time_sec).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            'image_width': cv_image.shape[1],
            'image_height': cv_image.shape[0],
            'encoding': 'bgr8'
        }
        
        with open(json_path, 'w') as json_file:
            json.dump(metadata, json_file, indent=4)
        
        self.get_logger().info(f'Saved {image_filename} and {json_filename}')
        
        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
