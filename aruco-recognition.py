#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import sys


calib_data = np.load('MultiMatrix.npz')

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# Importa a biblioteca ArUco
import cv2.aruco as aruco

# Define o ID dos ArUcos a serem detectados
aruco_id = 0
MARKER_SIZE = 2

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # receber as imagens da câmera
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_subscriber
        
        # receber as informações da câmera
        self.camera_info_subscriber = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.camera_info_subscriber
        
        # Configura o publicador para publicar a imagem com os ArUcos detectados
        self.image_publisher = self.create_publisher(Image, '/aruco_detector/output_image', 10)
        
        # Inicializa o objeto cv_bridge
        self.bridge = CvBridge()
        
        # Inicializa a matriz de calibração da câmera (será atualizada na callback de informações da câmera)
        self.camera_matrix = None
        self.dist_coeffs = None
    def image_callback(self, msg):
        # Converte a imagem ROS em uma imagem OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Converte a imagem para tons de cinza
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Inicializa o detector de ArUcos com as configurações padrão
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        
        # Detecta os ArUcos na imagem
        marker_corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Se pelo menos um ArUco for detectado
        if ids is not None:
            # Procura o ArUco com o ID especificado
            index = np.where(ids == aruco_id)[0]
            print("oi")

            if marker_corners:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
                )
                total_markers = range(0, ids.size)
            
            
            if len(index) > 0:
                # Desenha o contorno do ArUco encontrado
                for ids, corners, i in zip(ids, marker_corners, total_markers):
                    cv2.polylines(
                        cv_image, [corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA
                    )
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = tuple(corners[0].ravel())
                    top_left = tuple(corners[1].ravel())
                    bottom_right = tuple(corners[2].ravel())
                    bottom_left = tuple(corners[3].ravel())

                    #calcula a distancia.
                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )
                    
                    point = cv2.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                    cv2.putText(
                        cv_image,
                        f"id: {ids[0]} Dist: {round(distance, 2)}",
                        top_left,
                        cv2.FONT_HERSHEY_PLAIN,
                        2,
                        (255, 0, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        cv_image,
                        f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                        top_right,
                        cv2.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )

                
                # Publica a imagem com os ArUcos detectados
                output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                self.image_publisher.publish(output_msg)
        
    def camera_info_callback(self, msg):
        # Atualiza a matriz de calibração da câmera
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        
def main(args=None):
    while True:
        rclpy.init(args=args)
        # Cria o nó do detector de ArUcos
        aruco_detector = ArUcoDetector()
        try:
            rclpy.spin(aruco_detector)
        except KeyboardInterrupt:
        # Encerra o programa caso o usuário pressione Ctrl+C
            aruco_detector.destroy_node()
            rclpy.shutdown()
            break
        
if __name__ == '__main__':
	main()