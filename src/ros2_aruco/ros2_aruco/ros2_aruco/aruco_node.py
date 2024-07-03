"""
original repo: https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco
Author: Nathan Sprague
Changed for opencv 4.9 by Vadim Shtein
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations as tf
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers, DroneCommand
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32MultiArray
from colorama import Fore


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Added to produce transform from camera link to world link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.marker_size = self.get_parameter("marker_size").value
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = self.get_parameter("image_topic").value
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = self.get_parameter("camera_info_topic").value
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = self.get_parameter("camera_frame").value

        # Make sure we have a valid dictionary id:
        try:
            # Change to opencv 4.9
            # self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(int(dictionary_id_name))
            self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            if not isinstance(self.aruco_dictionary, cv2.aruco.Dictionary):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT_")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
        # self.marker_detection = self.create_publisher(String, '/aruco_detected', 10)
        self.marker_detection = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
        self.distortion = np.array(info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        if marker_ids is not None:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.intrinsic_mat, self.distortion
            )
            for i, marker_id in enumerate(marker_ids):
                # Extracting marker corners and center
                marker_corners = corners[i][0]
                center = np.mean(marker_corners, axis=0)

                # Printing corner coordinates
                # print(f"Marker ID {marker_id}:")
                for corner_idx, corner in enumerate(marker_corners):
                    print(f"  Corner {corner_idx + 1}: ({corner[0]}, {corner[1]})")

                # Printing center coordinates
                self.get_logger().info(f"{Fore.RED} Center: ({center[0]}, {center[1]}{Fore.RESET})")
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                quat = tf.quaternion_from_matrix(
                    np.vstack((np.hstack((rot_matrix, tvecs[i].reshape(3, 1))), np.array([0, 0, 0, 1]).reshape(1, 4))))

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])
                drone_command_x_y = Float32MultiArray()
                # drone_command_x_y.data = [-pose.position.x, -pose.position.y]
                drone_command_x_y.data = [pose.position.x, pose.position.y]


            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
            self.marker_detection.publish(drone_command_x_y)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# import rclpy
# import rclpy.node
# from rclpy.qos import qos_profile_sensor_data
# from cv_bridge import CvBridge
# import numpy as np
# import cv2
# import tf_transformations as tf
# from sensor_msgs.msg import CameraInfo, Image
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener
# from tf2_ros import TransformException
# from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
# from ros2_aruco_interfaces.msg import ArucoMarkers
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType
# from std_msgs.msg import Float32MultiArray
# import tf2_geometry_msgs
#
#
# class ArucoNode(rclpy.node.Node):
#     def __init__(self):
#         super().__init__("aruco_node")
#
#         # Added to produce transform from camera link to world link
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#
#         # Declare and read parameters
#         self.declare_parameter(
#             name="marker_size",
#             value=0.0625,
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_DOUBLE,
#                 description="Size of the markers in meters.",
#             ),
#         )
#
#         self.declare_parameter(
#             name="aruco_dictionary_id",
#             value="DICT_5X5_250",
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_STRING,
#                 description="Dictionary that was used to generate markers.",
#             ),
#         )
#
#         self.declare_parameter(
#             name="image_topic",
#             value="/camera/image_raw",
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_STRING,
#                 description="Image topic to subscribe to.",
#             ),
#         )
#
#         self.declare_parameter(
#             name="camera_info_topic",
#             value="/camera/camera_info",
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_STRING,
#                 description="Camera info topic to subscribe to.",
#             ),
#         )
#
#         self.declare_parameter(
#             name="camera_frame",
#             value="camera_link",
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_STRING,
#                 description="Camera optical frame to use.",
#             ),
#         )
#
#         self.declare_parameter(
#             name="world_frame",
#             value="world",
#             descriptor=ParameterDescriptor(
#                 type=ParameterType.PARAMETER_STRING,
#                 description="World frame to use.",
#             ),
#         )
#
#         self.marker_size = self.get_parameter("marker_size").value
#         self.get_logger().info(f"Marker size: {self.marker_size}")
#
#         dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
#         self.get_logger().info(f"Marker type: {dictionary_id_name}")
#
#         image_topic = self.get_parameter("image_topic").value
#         self.get_logger().info(f"Image topic: {image_topic}")
#
#         info_topic = self.get_parameter("camera_info_topic").value
#         self.get_logger().info(f"Image info topic: {info_topic}")
#
#         self.camera_frame = "camera_link"
#         self.world_frame = "world"
#         # self.world_frame = self.get_parameter("ground_plane").value
#
#         # Make sure we have a valid dictionary id:
#         try:
#             self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#             if not isinstance(self.aruco_dictionary, cv2.aruco.Dictionary):
#                 raise AttributeError
#         except AttributeError:
#             self.get_logger().error(
#                 "bad aruco_dictionary_id: {}".format(dictionary_id_name)
#             )
#             options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT_")])
#             self.get_logger().error("valid options: {}".format(options))
#
#         # Set up subscriptions
#         self.info_sub = self.create_subscription(
#             CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
#         )
#
#         self.image_sub = self.create_subscription(
#             Image, image_topic, self.image_callback, qos_profile_sensor_data
#         )
#
#         # Set up publishers
#         self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
#         self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
#         self.marker_detection = self.create_publisher(Float32MultiArray, '/aruco_detected', 10)
#
#         # Set up fields for camera parameters
#         self.info_msg = None
#         self.intrinsic_mat = None
#         self.distortion = None
#
#         self.aruco_parameters = cv2.aruco.DetectorParameters()
#         self.bridge = CvBridge()
#
#     def info_callback(self, info_msg):
#         self.info_msg = info_msg
#         self.intrinsic_mat = np.reshape(np.array(info_msg.k), (3, 3))
#         self.distortion = np.array(info_msg.d)
#         self.destroy_subscription(self.info_sub)
#
#     def image_callback(self, img_msg):
#         if self.info_msg is None:
#             self.get_logger().warn("No camera info has been received!")
#             return
#
#         cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
#         markers = ArucoMarkers()
#         pose_array = PoseArray()
#         if self.camera_frame == "":
#             markers.header.frame_id = self.info_msg.header.frame_id
#             pose_array.header.frame_id = self.info_msg.header.frame_id
#         else:
#             markers.header.frame_id = self.camera_frame
#             pose_array.header.frame_id = self.camera_frame
#
#         markers.header.stamp = img_msg.header.stamp
#         pose_array.header.stamp = img_msg.header.stamp
#
#         corners, marker_ids, rejected = cv2.aruco.detectMarkers(
#             cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
#         )
#         if marker_ids is not None:
#             rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
#                 corners, self.marker_size, self.intrinsic_mat, self.distortion
#             )
#             for i, marker_id in enumerate(marker_ids):
#                 pose = Pose()
#                 pose.position.x = tvecs[i][0][0]
#                 pose.position.y = tvecs[i][0][1]
#                 pose.position.z = tvecs[i][0][2]
#
#                 rot_matrix, _ = cv2.Rodrigues(rvecs[i])
#                 quat = tf.quaternion_from_matrix(
#                     np.vstack((np.hstack((rot_matrix, tvecs[i].reshape(3, 1))), np.array([0, 0, 0, 1]).reshape(1, 4))))
#
#                 pose.orientation.x = quat[0]
#                 pose.orientation.y = quat[1]
#                 pose.orientation.z = quat[2]
#                 pose.orientation.w = quat[3]
#
#                 # Apply the transformation from camera_frame to world_frame
#                 pose_world = self.transform_pose_to_world_frame(pose)
#
#                 pose_array.poses.append(pose)
#                 markers.poses.append(pose)
#                 markers.marker_ids.append(marker_id[0])
#                 drone_command_x_y = Float32MultiArray()
#                 drone_command_x_y.data = [-pose.position.x, -pose.position.y]
#
#             self.poses_pub.publish(pose_array)
#             self.markers_pub.publish(markers)
#             self.marker_detection.publish(drone_command_x_y)
#
#     def transform_pose_to_world_frame(self, pose):
#         # Create a PoseStamped object for transformation
#         pose_stamped = PoseStamped()
#         pose_stamped.pose = pose
#         pose_stamped.header.frame_id = self.camera_frame
#         pose_stamped.header.stamp = self.get_clock().now().to_msg()
#
#         try:
#             # Get the transformation from camera_frame to world_frame
#             transform = self.tf_buffer.lookup_transform(self.world_frame, self.camera_frame, rclpy.time.Time())
#             # transform = self.tf_buffer.lookup_transform("world", "camera_link", rclpy.time.Time())
#
#             pose_world = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
#             return pose_world.pose
#         except TransformException as ex:
#             self.get_logger().error(f"Could not transform {self.camera_frame} to {self.world_frame}: {ex}")
#             return pose
#
#
# def main():
#     rclpy.init()
#     node = ArucoNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#
#
# if __name__ == "__main__":
#     main()
