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
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped
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

#from pose_msgs.msg import poseDictionaryMsg

# Importa a biblioteca ArUco
import cv2.aruco as aruco
# Define o ID dos ArUcos a serem detectados
aruco_id = 0

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # receber as imagens da câmera
        self.create_subscription(Image, '/freedom_vehicle/camera/image_raw', self.image_callback, 10)

        # receber as informações da câmera
        self.create_subscription(CameraInfo, '/freedom_vehicle/camera/camera_info', self.camera_info_callback, 10)
        print('aquieuto')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        print('aquieuto2')

        
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

        self.pos_odom = Point()
        self.pos_odom.x = 0.0
        self.pos_odom.y = 0.0
        self.pos_odom.z = 0.0

    def image_callback(self, msg : Image):
        marker_size = 0
        # Converte a imagem ROS em uma imagem OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
             # Converte a imagem para tons de fe(self.cv_image_pixel.copy(), (500, 400), interpolation = cv2.INTER_AREA))
            # cv2.imwrite(dirPath + '/pixel_test.png', cv2.resize(self.cv_image_pixel.copy(), (500, 400), interpolation = cv2.INTER_AREA))
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detecta os ArUcos na imagem
        marker_corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if len(marker_corners) > 0:
            print('adas', self.distIDs.keys)
            
            # Procura o ArUco com o ID especificado
            for i in range(len(marker_corners)):
                aruco_id = ids[i][0]
                marker_corner = marker_corners[i]
                if aruco_id in self.distIDs.keys():
                    print("aqui")
                    marker_size = self.distIDs[aruco_id]
                    cv_image = self.aruco_detection(marker_corner, marker_size, msg, cv_image, aruco_id, self.create_transform_matrix_3d)
                    ids_presentes = [item['id'] for item in self.aruco_info]
                    print('id', aruco_id)
                    if aruco_id in ids_presentes:
                        for item in self.aruco_info:
                            if item['id'] == aruco_id:
                                aruco_data = item
                                break
                        self.pos.x = aruco_data['pos_x']
                        self.pos.y = aruco_data['pos_y'] 
                        self.pos.z = aruco_data['pos_z']         

                    # Publica a imagem com os ArUcos detectados
                    output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    self.image_publisher.publish(output_msg)

                try:
                    cv2.imshow("O(t)_output", cv2.resize(cv_image, (500, 400), interpolation = cv2.INTER_AREA))
                    # cv2.imwrite(dirPath + '/pixel_test.png', cv2.resize(self.cv_image_pixel.copy(), (500, 400), interpolation = cv2.INTER_AREA))
                    cv2.waitKey(3)
                except CvBridgeError as e:
                    print(e)

    def create_transform_matrix_3d(self, angle_x, angle_y, angle_z, tx, ty, tz):
        Rx = np.array([[1, 0, 0, 0],
                    [0, np.cos(angle_x), -np.sin(angle_x), 0],
                    [0, np.sin(angle_x), np.cos(angle_x), 0],
                    [0, 0, 0, 1]])
        
        Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle_y), 0, np.cos(angle_y), 0],
                    [0, 0, 0, 1]])
        
        Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0, 0],
                    [np.sin(angle_z), np.cos(angle_z), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        translation_vector = np.array([[tx], [ty], [tz], [1]])

        transform_matrix = Rz @ Ry @ Rx
        transform_matrix[:3, 3] = translation_vector.flatten()[:3]

        return transform_matrix

    def aruco_detection(self, marker_corner, marker_size, msg, cv_image, aruco_id, create_transform_matrix_3d):
        print('aruco_detection')

        try:
            # Obtenha a transformação atual entre os quadros
            transform = self.tf_buffer.lookup_transform('odom', 'camera_link', rclpy.time.Time(), timeout=rclpy.time.Duration(seconds=1))
            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            rotation = np.array([transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w])

            transform_matrix = self.create_transform_matrix_3d(0, 0, 0, translation[0], translation[1], translation[2])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("Erro ao obter a transformação:", e)


        _, tVec, _ = aruco.estimatePoseSingleMarkers(
                        marker_corner, marker_size, self.camera_matrix, self.dist_coef
                        )

        distance = np.sqrt(tVec[0][0][2] ** 2 + tVec[0][0][0] ** 2 + tVec[0][0][1] ** 2)
        
        tVec_odom = np.dot(transform_matrix, distance)
        
        aruco_pos_cam = Point()
        aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z = tVec[0][0][2]/100, -tVec[0][0][0]/100, -tVec[0][0][1]/100

        aruco_pos_cam_np = np.array([aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z, 1])

        aruco_ori = Quaternion()
        aruco_ori.w = 1.

        aruco_pos_odom = np.dot(transform_matrix, aruco_pos_cam_np)
        print('aruco_pos_odom', aruco_pos_odom)


        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"marker_{aruco_id}"

        t.transform.translation.x = aruco_pos_cam.x
        t.transform.translation.y = aruco_pos_cam.y
        t.transform.translation.z = aruco_pos_cam.z
        t.transform.rotation = aruco_ori

        self.tf_broadcaster.sendTransform(t)
        print('aruco_camera_translation', t.transform.translation)
        print('aruco_camera_rotation', t.transform.rotation)

        # t_cambase = self.tf_buffer.lookup_transform(
        # 'camera_link',
        # 'base_link',
        # rclpy.time.Time())
        
        # # Publica a transformação
        # self.tf_broadcaster.sendTransform(t_cambase) 
        # self.print_camera_position(t_cambase)
        # #print('uepa', t_cambase)


        # # Vetor de rotação da transformada t_cambase
        # rotation_w = t_cambase.transform.rotation.w
        # rotation_z = t_cambase.transform.rotation.z
        # rotation_x = t_cambase.transform.rotation.x
        # rotation_y = t_cambase.transform.rotation.y

        # # Vetor de translação da transformada t_cambase

        # translation_x = t_cambase.transform.translation.x
        # translation_y = t_cambase.transform.translation.y
        # translation_z = t_cambase.transform.translation.z

        

        # camera_odom = Quaternion(x=rotation_x, y=rotation_y, z=rotation_z, w=rotation_w)
        # camera_odom.x = self.pos_odom.x + translation_x
        # camera_odom.y = self.pos_odom.y + translation_y
        # camera_odom.z = self.pos_odom.z + translation_z
        aruco_world = np.array([self.pos.x, self.pos.y, self.pos.z, 1.0])
        aruco_cam= np.array([aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z, 1.0])

        odom_map = Point()
        odom_map.x = aruco_world[0] - self.pos.x
        odom_map.y = aruco_world[1] - self.pos.y
        odom_map.z = aruco_world[2] - self.pos.z
        

        odom_map = Point()
        odom_map.x = self.pos.x - (t.transform.translation.x + transform.transform.translation.x)
        odom_map.y = self.pos.y - (t.transform.translation.y + transform.transform.translation.y)
        odom_map.z = self.pos.z - (t.transform.translation.z + transform.transform.translation.z)

        #print('map', odom_map)
        #print('camera_odom', camera_odom)
        #print('posicao', self.pos.x)
        #print('aruco', aruco_pos_cam.x)
        #print('t_cambase',t_cambase.transform.translation.x)
        # Desenha o contorno do ArUco encontrado
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
            f"x:{round(aruco_pos_cam.x,1)} y: {round(aruco_pos_cam.y,1)} ",
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