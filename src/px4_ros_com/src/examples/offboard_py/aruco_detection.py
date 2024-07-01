#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge, CvBridgeError
from colorama import Fore


class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.image_subscription = self.create_subscription(Image, '/drone/camera/image_raw',
                                                           self.image_subscription_callback, 10)

        self.detection_publisher = self.create_publisher(String, '/aruco_detection', 10)
        self.bridge = CvBridge()

        self.desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"

        # The different ArUco dictionaries built into the OpenCV library.
        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }

    def image_subscription_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"{Fore.CYAN}CV image received{Fore.RESET}")
        except CvBridgeError as e:
            print(f"CvBridgeError: {e}")
            return

        # Detect ArUco markers
        arucoDict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.desired_aruco_dictionary])
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)

        self.get_logger().info(f"{Fore.YELLOW}Aruco detection....{Fore.RESET}")
        # Publish detection results
        if len(corners) > 0:
            ids_str = ",".join(str(id) for id in ids)
            self.get_logger().info(f"Detected ArUco markers: {ids_str}")
            self.detection_publisher.publish(String(data=ids_str))


def main(args=None):
    rclpy.init(args=args)
    aruco = ArucoDetection()
    rclpy.spin(aruco)
    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

