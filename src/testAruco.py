#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import sys
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

calib_data = np.load('MultiMatrix.npz')

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

# Import the ArUco library
import cv2.aruco as aruco

# Define the ID of the ArUco marker to be detected
aruco_id = 0
MARKER_SIZE = 2

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Receive camera images
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_subscriber
        
        # Receive camera information
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.camera_info_subscriber
        
        # Configure the publisher to publish the image with detected ArUcos
        self.image_publisher = self.create_publisher(Image, '/aruco_detector/output_image', 10)
        
        # Initialize the cv_bridge object
        self.bridge = CvBridge()
        
        # Initialize the camera calibration matrix (will be updated in the camera info callback)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize TF-related objects
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
    def image_callback(self, msg):
        # Convert the ROS image to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Initialize the ArUco detector with default settings
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
        parameters = aruco.DetectorParameters_create()
        
        # Detect the ArUcos in the image
        marker_corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # If at least one ArUco marker is detected
        if ids is not None:
            # Look for the ArUco with the specified ID
            index = np.where(ids == aruco_id)[0]
            print("oi")

            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
                )
                total_markers = range(0, ids.size)
            
            if len(index) > 0:
                # Retrieve the transform between the camera and the robot's base_link
                try:
                    transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, rclpy.time.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    return
                
                # Extract the translation and rotation components from the transform
                tvec_robot = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
                rvec_robot = np.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
                
                # Transform the ArUco pose into the robot's coordinate system
                aruco_pos_robot = []
                for i in range(ids.size):
                    aruco_pos_cam = np.array([tVec[i][0][0], tVec[i][0][1], tVec[i][0][2]])
                    aruco_pos_robot.append(tf2_geometry_msgs.do_transform_pose(geometry_msgs.msg.PoseStamped(
                        header=transform.header,
                        pose=geometry_msgs.msg.Pose(
                            position=geometry_msgs.msg.Point(*aruco_pos_cam),
                            orientation=geometry_msgs.msg.Quaternion(0, 0, 0, 1)
                        )
                    ), transform).pose.position)
                
                # Draw the contour of the detected ArUco
                for ids, corners, i in zip(ids, marker_corners, total_markers):
                    cv2.polylines(cv_image, [corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = tuple(corners[0].ravel())
                    top_left = tuple(corners[1].ravel())
                    bottom_right = tuple(corners[2].ravel())
                    bottom_left = tuple(corners[3].ravel())

                    # Calculate the distance
                    distance = np.sqrt(tvec[i][0][2] ** 2 + tvec[i][0][0] ** 2 + tvec[i][0][1] ** 2)
                    
                    point = cv2.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                    cv2.putText(cv_image, f"id: {ids[0]} Dist: {round(distance, 2)}", top_left, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(cv_image, f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ", top_right, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)
                
                # Publish the image with detected ArUcos
                output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.image_publisher.publish(output_msg)
    
    def camera_info_callback(self, msg):
        # Update the camera calibration matrix
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        
def main(args=None):
    while True:
        rclpy.init(args=args)
        # Create the ArUco detector node
        aruco_detector = ArUcoDetector()
        try:
            rclpy.spin(aruco_detector)
        except KeyboardInterrupt:
            # Terminate the program if the user presses Ctrl+C
            aruco_detector.destroy_node()
            rclpy.shutdown()
            break

if __name__ == '__main__':
    main()