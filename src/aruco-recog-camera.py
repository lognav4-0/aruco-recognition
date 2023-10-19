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
#from pose_msgs.msg import poseDictionaryMsg

# Importa a biblioteca ArUco
import cv2.aruco as aruco
import time

calib_data_path = "./calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

# Define o ID dos ArUcos a serem detectados


class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # receber as imagens da câmera
        self.create_subscription(Image, '/camera_test', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_test_info', self.camera_info_callback, 10)

        
        # Configura o publicador para publicar a imagem com os ArUcos detectados
        self.image_publisher = self.create_publisher(Image, '/aruco_detector/output_image', 10)

        self.publisher_stamp = self.create_publisher(PoseStamped, 'pose_stamped_topic', 10)
        
        # Inicializa o objeto cv_bridge
        self.bridge = CvBridge()

        # Crie um dicionário para armazenar os tamanhos de cada ArUco com base no ID
        self.aruco_sizes = {}
        #Cria o dicionario com as distancias p/ID
        self.distIDs = {}
        self.valor_ate_50 = 8
        for i in range(0, 1):
            self.distIDs[i] = self.valor_ate_50

        # Definir outro valor igual para as chaves de 51 a 100
        self.valor_ate_100 = 5
        for i in range(1, 2):
            self.distIDs[i] = self.valor_ate_100


        print('asdas', self.distIDs)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
            }
        aruco_type = "DICT_4X4_1000"
        self.aruco_dict = cv2.aruco.Dictionary_get(arucoDicts[aruco_type])
        self.parameters = aruco.DetectorParameters_create()

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
            # Procura o ArUco com o ID especificado
            for i in range(len(marker_corners)):
                aruco_id = ids[i][0]
                marker_corner = marker_corners[i]
                if aruco_id in self.distIDs.keys():
                    marker_size = self.distIDs[aruco_id]
                    print('id', aruco_id)
                    print('marker', marker_size)

                    cv_image = self.aruco_detection(marker_corner, marker_size, msg, cv_image, aruco_id)

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
        _, tVec, _ = aruco.estimatePoseSingleMarkers(
                        marker_corner, marker_size, cam_mat, dist_coef
                        )

        distance = np.sqrt(tVec[0][0][2] ** 2 + tVec[0][0][0] ** 2 + tVec[0][0][1] ** 2)
        
        aruco_pos_cam = Point()
        aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z = tVec[0][0][0], tVec[0][0][1], tVec[0][0][2]
        aruco_ori = Quaternion()
        aruco_ori.w = 1.    
        print('-------')
    
        # Desenha o contorno do ArUco encontrado
        cv2.polylines(
            cv_image, [marker_corner.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA
        )
        marker_corner = marker_corner.reshape(4, 2)
        marker_corner = marker_corner.astype(int)
        top_right = tuple(marker_corner[0].ravel())
        top_left = tuple(marker_corner[1].ravel())

        #calcula a distancia.
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
        # Atualiza a matriz de calibração da câmera
        self.cam_mat = np.array(msg.k).reshape((3, 3))
        self.dist_coef = np.array(msg.d)
        
def main(args=None):
    print('hello')
    rclpy.init(args=args)

    # Cria o nó do detector de ArUcos
    aruco_detector = ArUcoDetector()
    while rclpy.ok():
        rclpy.spin_once(aruco_detector)
    # Encerra o programa caso o usuário pressione Ctrl+C
    aruco_detector.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
	main()