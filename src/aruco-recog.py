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
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry

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

        with open('arucos_infos.json', 'r') as arquivo_json:
            self.aruco_info = json.load(arquivo_json)
        
         # Crie um dicionário para armazenar os tamanhos de cada ArUco com base no ID
        self.aruco_sizes = {}
        
        #Cria o dicionario com as distancias p/ID
        self.distIDs = {}
        self.valor_ate_50 = 7.14
        for i in range(0, 1):
            self.distIDs[i] = self.valor_ate_50

        # Definir outro valor igual para as chaves de 51 a 100
        self.valor_ate_100 = 7.14
        for i in range(1, 6):
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

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

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
                    cv_image = self.aruco_detection(marker_corner, marker_size, msg, cv_image, aruco_id)
                    ids_presentes = [item['id'] for item in self.aruco_info]
                    print('id', aruco_id)
                    if aruco_id in ids_presentes:
                        for item in self.aruco_info:
                            if item['id'] == aruco_id:
                                aruco_data = item
                                break                        

                    # Publica a imagem com os ArUcos detectados
                    output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    self.image_publisher.publish(output_msg)

                try:
                    cv2.imshow("O(t)_output", cv2.resize(cv_image, (500, 400), interpolation = cv2.INTER_AREA))
                    # cv2.imwrite(dirPath + '/pixel_test.png', cv2.resize(self.cv_image_pixel.copy(), (500, 400), interpolation = cv2.INTER_AREA))
                    cv2.waitKey(3)
                except CvBridgeError as e:
                    print(e)

    def aruco_detection(self, marker_corner, marker_size, msg, cv_image, aruco_id):
        print('aruco_detection')
        _, tVec, _ = aruco.estimatePoseSingleMarkers(
                        marker_corner, marker_size, self.camera_matrix, self.dist_coef
                        )

        distance = np.sqrt(tVec[0][0][2] ** 2 + tVec[0][0][0] ** 2 + tVec[0][0][1] ** 2)
        
        
        aruco_pos_cam = Point()
        aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z = tVec[0][0][2]/100, -tVec[0][0][0]/100, -tVec[0][0][1]/100

        aruco_ori = Quaternion()
        aruco_ori.w = 1.


        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_link'
        t.child_frame_id = f"marker_{aruco_id}"

        t.transform.translation.x = aruco_pos_cam.x
        t.transform.translation.y = aruco_pos_cam.y
        t.transform.translation.z = aruco_pos_cam.z
        t.transform.rotation = aruco_ori

        self.tf_broadcaster.sendTransform(t)


        t_cambase = self.tf_buffer.lookup_transform(
        'base_link',
        'camera_link',
        rclpy.time.Time())
        
        print('opa', t_cambase.transform.translation.x)        
        # Publica a transformação
        self.tf_broadcaster.sendTransform(t_cambase) 
        self.print_camera_position(t_cambase)


        print('odom', self.pos_x, self.pos_y, self.pos_z)

        map = Point()
        map.x = self.pos_x - t_cambase.transform.translation.x
        map.y = self.pos_y - t_cambase.transform.translation.y
        map.z = self.pos_z - t_cambase.transform.translation.z
        

        print('map', map)

        # Desenha o contorno do ArUco encontrado
        cv2.polylines(
            cv_image, [marker_corner.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA
        )
        marker_corner = marker_corner.reshape(4, 2)
        marker_corner = marker_corner.astype(int)
        top_right = tuple(marker_corner[0].ravel())
        top_left = tuple(marker_corner[1].ravel())



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
            f"x:{round(tVec[0][0][0],1)} y: {round(tVec[0][0][1],1)} ",
            top_right,
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
        print('msg', msg.pose.pose.position.x)
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z

            
    def print_camera_position(self, *transforms):
        for transform in transforms:
            self.get_logger().info(f"aruco position for cam:")
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