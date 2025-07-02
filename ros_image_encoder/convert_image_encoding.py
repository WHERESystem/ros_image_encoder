#!/usr/bin/env python3

import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter(Node):
    def __init__(self, topic):
        super().__init__('image_converter_' + topic.replace('/', '_'))

        self.bridge = CvBridge()
        self.cv_image = None

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.callback,
            10)

        self.publisher_opencv = self.create_publisher(
            Image,
            topic + '_bgr_opencv',
            10)

        self.publisher_rgb = self.create_publisher(
            Image,
            topic + '_rgb',
            10)

        self.get_logger().info(f"Initialized ImageConverter for topic: {topic}")

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            yolo_img = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

            self.publisher_opencv.publish(self.bridge.cv2_to_imgmsg(yolo_img, "bgr8"))
            self.publisher_rgb.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)

    # Create a temporary node to fetch parameters
    param_node = Node('param_loader')
    param_node.declare_parameter('frame_rate', rclpy.Parameter.Type.DOUBLE)
    param_node.declare_parameter('topic_list',rclpy.Parameter.Type.STRING_ARRAY)

    frame_rate = param_node.get_parameter('frame_rate').value
    topic_list = param_node.get_parameter('topic_list').value

    param_node.get_logger().info(str(frame_rate))

    param_node.destroy_node()

    converters = []
    for topic in topic_list:
        converter = ImageConverter(topic)
        converters.append(converter)

    try:
        last_time = time.time()
        while rclpy.ok():
            for converter in converters:
                rclpy.spin_once(converter, timeout_sec=0.1)
            elapsed = time.time() - last_time
            sleep_time = max(0.0, 1.0 / frame_rate - elapsed)
            time.sleep(sleep_time)
            last_time = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        for converter in converters:
            converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
