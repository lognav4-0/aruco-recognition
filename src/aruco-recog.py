#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Twist
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
import math
import json
import tf2_ros
import transforms3d as t3d
import tf2_py as tf2
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry
import os
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as R


#from pose_msgs.msg import poseDictionaryMsg

# Importa a biblioteca ArUco
import cv2.aruco as aruco
# Define o ID dos ArUcos a serem detectados
aruco_id = 0

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.camera1_subscription = self.create_subscription(Image, '/freedom_vehicle/camera/image_raw', self.camera1_image_callback, 10)
        self.camera2_subscription = self.create_subscription(Image, '/freedom_vehicle/camera2/image_raw', self.camera2_image_callback, 10)


        self.create_subscription(CameraInfo, '/freedom_vehicle/camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # Configura o publicador para publicar a imagem com os ArUcos detectados
        self.image_publisher = self.create_publisher(Image, '/aruco_detector/output_image', 10)
        #self.positionWorld_publisher = self.create_publisher(poseDictionaryMsg, 'robot_pose_topic', 10)

        self.publisher_stamp = self.create_publisher(PoseStamped, 'pose_stamped_topic', 10)
        
        # Inicializa o objeto cv_bridge
        self.bridge = CvBridge()

        script_dir = os.path.dirname(__file__)
        #path = os.path.join(script_dir, '..', '..', '..', 'src', 'arucos_infos.json')
        
        path =  '/home/lognav/lognav_ws/src/aruco_recognition/src/arucos_infos.json'
        with open(path, 'r') as arquivo_json:
            self.aruco_info = json.load(arquivo_json)
        
         # Crie um dicionário para armazenar os tamanhos de cada ArUco com base no ID
        self.aruco_sizes = {}
        
        #Cria o dicionario com as distancias p/ID
        self.distIDs = {}
        self.valor_ate_50 = 7.14
        for i in range(0, 100):
            self.distIDs[i] = self.valor_ate_50

        # Definir outro valor igual para as chaves de 51 a 100
        self.valor_ate_100 = 7.14
        for i in range(101, 200):
            self.distIDs[i] = self.valor_ate_100


        # Inicializa a matriz de calibração da câmera (será atualizada na callback de informações da câmera)
        self.camera_matrix = None
        self.dist_coef = None

        #self.position_tf_publisher = tf2_ros.Transform
        arucoDicts = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
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
            "DICT_7X7_50": cv2.aruco.DICT_6X6_50,
            "DICT_7X7_100": cv2.aruco.DICT_6X6_100,
            "DICT_7X7_250": cv2.aruco.DICT_6X6_250,
            "DICT_7X7_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
            }
        aruco_type = "DICT_4X4_50"
        self.aruco_dict = cv2.aruco.Dictionary_get(arucoDicts[aruco_type])
        self.parameters = aruco.DetectorParameters_create()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pos =  Point()  
        self.pos.x = 0.0
        self.pos.y = 0.0
        self.pos.z = 0.0
        self.yaw = 0.0

        self.pos_odom = Point()
        self.pos_odom.x = 0.0
        self.pos_odom.y = 0.0
        self.pos_odom.z = 0.0

    def camera1_image_callback(self, msg: Image):
        self.process_image(msg, camera_id=1)
    def camera2_image_callback(self, msg: Image):
        self.process_image(msg, camera_id=2)

    def process_image(self, msg: Image, camera_id: int):
        marker_size = 0
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        marker_corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if len(marker_corners) > 0:
            for i in range(len(marker_corners)):
                aruco_id = ids[i][0]
                marker_corner = marker_corners[i]
                if aruco_id in self.distIDs.keys():
                    marker_size = self.distIDs[aruco_id]
                    cv_image = self.aruco_detection(marker_corner, marker_size, msg, cv_image, aruco_id, self.create_transform_matrix_3d)
                    ids_presentes = [item['id'] for item in self.aruco_info]
                    if aruco_id in ids_presentes:
                        for item in self.aruco_info:
                            if item['id'] == aruco_id:
                                aruco_data = item
                                break
                        self.pos.x = np.float64(aruco_data['pos_x'])
                        self.pos.y = np.float64(aruco_data['pos_y'])
                        self.pos.z = np.float64(aruco_data['pos_z'])
                        self.yaw = np.float64(aruco_data['yaw'])

                    output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    self.image_publisher.publish(output_msg)

                    while camera_id == 2 and aruco_id == 1:
                        twist = Twist()
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.linear.z = 0.0
                        twist.angular.x = 0.0
                        twist.angular.y = 0.0
                        twist.angular.z = 0.0
                        self.pub.publish(twist)

                try:
                    cv2.imshow("O(t)_output", cv2.resize(cv_image, (500, 400), interpolation=cv2.INTER_AREA))
                    cv2.waitKey(3)
                except CvBridgeError as e:
                    print(e)

    def create_transform_matrix_3d(self, angle_x, angle_y, angle_z, tx, ty, tz):
        Rx = np.array([[1, 0, 0, 0],
                    [0, np.cos(angle_x), np.sin(angle_x), 0],
                    [0, -np.sin(angle_x), np.cos(angle_x), 0],
                    [0, 0, 0, 1]])
        
        Ry = np.array([[np.cos(angle_y), 0, -np.sin(angle_y), 0],
                    [0, 1, 0, 0],
                    [np.sin(angle_y), 0, np.cos(angle_y), 0],
                    [0, 0, 0, 1]])
        
        Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0, 0],
                    [-np.sin(angle_z), np.cos(angle_z), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        translation_vector = np.array([[tx], [ty], [tz], [1]])

        transform_matrix = Rz @ Ry @ Rx
        transform_matrix[:3, 3] = translation_vector.flatten()[:3]

        return transform_matrix

    def aruco_detection(self, marker_corner, marker_size, msg, cv_image, aruco_id, create_transform_matrix_3d):
 
        t_mundo_aruco = self.create_transform_matrix_3d(0, 0, self.yaw, self.pos.x, self.pos.y, self.pos.z)

        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation = np.array([transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w])
            
            transform_euler = R.from_quat([rotation[0], rotation[1], rotation[2], rotation[3]])
            transform_euler = transform_euler.as_euler('xyz', degrees=False)

            t_base_camera = self.create_transform_matrix_3d(transform_euler[0], transform_euler[1], transform_euler[2], translation[0], translation[1], translation[2])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Erro ao obter a transformação:", e)


        marker_corner_array = np.array(marker_corner).astype(np.float32)
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corner_array, marker_size, self.camera_matrix, self.dist_coef)
        distance = np.sqrt(tVec[0][0][2] ** 2 + tVec[0][0][0] ** 2 + tVec[0][0][1] ** 2)

        
        t_aruco_camera = create_transform_matrix_3d(rVec[0][0][2], rVec[0][0][0], rVec[0][0][1], 
                                                        tVec[0][0][2]/100, -tVec[0][0][0]/100, 
                                                        -tVec[0][0][1]/100)
        #Posição do ArUco no mundo
        O0_0 = np.array([[0, 0, 0, 1]]).T       
        O1_0 = t_mundo_aruco @ O0_0
        print('O1_0', O1_0)

        #O2_2 = np.array([[0, 0, 0, 1]]).T
        #O1_2 = transform_matrix @ O2_2

        #Posição do Aruco para a câmera
        O2_2 = np.array([[0, 0, 0, 1]]).T
        O1_2 = t_aruco_camera @ O2_2
        print('O1_2', O1_2)

        t_aruco_camera_inv = np.linalg.inv(t_aruco_camera)
        print('t_aruco_camera_inv', t_aruco_camera_inv, t_aruco_camera)

        T20 = t_mundo_aruco @ t_aruco_camera_inv 
        print('T20', T20)
        print('ground_truth', [0.03, -1.5855, 0.93])
        #sla = T_mundo_aruco_final @ t_final @ transform_matrix
        #print('sla', sla)
        #print('ground_truth', self.pos_odom)
        
        cv2.polylines(
            cv_image, [marker_corner.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA
        )
        marker_corner = marker_corner.reshape(4, 2)
        marker_corner = marker_corner.astype(int)
        top_right = tuple(marker_corner[0].ravel())
        top_left = tuple(marker_corner[1].ravel())
        bottom_left = tuple(marker_corner[2].ravel())
        bottom_right = tuple(marker_corner[3].ravel())



        cv2.putText(
            cv_image,
            f"id: {aruco_id} Dist: {round(distance, 2)}",
            top_left,
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            cv_image,
            f"x:{round(tVec[0][0][2]/100,1)} y: {round(-tVec[0][0][0],1)} ",
            bottom_right,
            cv2.FONT_HERSHEY_PLAIN,
            2,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        return cv_image

    def camera_info_callback(self, msg : CameraInfo):
        print('camera_info_callback')
        # Atualiza a matriz de calibração da câmera
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coef = np.array(msg.d)

    def odom_callback(self, msg : Odometry):
        self.pos_odom.x = msg.pose.pose.position.x
        self.pos_odom.y = msg.pose.pose.position.y
        self.pos_odom.z = msg.pose.pose.position.z

            
    def print_camera_position(self, *transforms):
        for transform in transforms:
            self.get_logger().info(f"Translation (x, y, z): {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
            
def main(args=None):
    print('aruco-recog')
    rclpy.init(args=args)

    # Cria o nó do detector de ArUcos
    aruco_detector = ArUcoDetector()
    
    rclpy.spin(aruco_detector)
    # Encerra o programa caso o usuário pressione Ctrl+C
    aruco_detector.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
	main()