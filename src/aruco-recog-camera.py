#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rclpy.qos import QoSProfile
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, TransformStamped, Twist
from std_msgs.msg import Header
import cv2.aruco as aruco
import threading
from pynput import keyboard

# Caminho para o arquivo de calibração
calib_data_path = "/home/freedom/freedom_ws/src/aruco_recognition/src/calib_data/MultiMatrix.npz"

# Carregar dados de calibração
calib_data = np.load(calib_data_path)
print("Calib Data Files:", calib_data.files)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Assinaturas para imagens e informações da câmera
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Publicadores para imagem processada e pose
        self.image_publisher = self.create_publisher(Image, '/aruco_detector/output_image', 10)
        self.publisher_stamp = self.create_publisher(PoseStamped, 'pose_stamped_topic', 10)

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, '/hoverboard_base_controller/cmd_vel_unstamped', qos)

        # Inicializa o objeto cv_bridge
        self.bridge = CvBridge()

        # Dicionário de tamanhos de ArUco e distâncias por ID
        self.distIDs = {0: 8, 1: 5}  # Exemplo simplificado
        print('Dist IDs:', self.distIDs)

        # Buffer e listener de transformação
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Configura o dicionário e parâmetros do detector ArUco
        arucoDicts = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_4X4_250": aruco.DICT_4X4_250,
            "DICT_4X4_1000": aruco.DICT_4X4_1000,
            "DICT_5X5_50": aruco.DICT_5X5_50,
            "DICT_5X5_100": aruco.DICT_5X5_100,
            "DICT_5X5_250": aruco.DICT_5X5_250,
            "DICT_5X5_1000": aruco.DICT_5X5_1000,
            "DICT_6X6_50": aruco.DICT_6X6_50,
            "DICT_6X6_100": aruco.DICT_6X6_100,
            "DICT_6X6_250": aruco.DICT_6X6_250,
            "DICT_6X6_1000": aruco.DICT_6X6_1000,
            "DICT_7X7_50": aruco.DICT_7X7_50,
            "DICT_7X7_100": aruco.DICT_7X7_100,
            "DICT_7X7_250": aruco.DICT_7X7_250,
            "DICT_7X7_1000": aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL
        }
        aruco_type = "DICT_4X4_1000"
        self.aruco_dict = aruco.Dictionary_get(arucoDicts[aruco_type])
        self.parameters = aruco.DetectorParameters_create()

        # Variáveis para controlar o estado do robô
        self.paused = False
        self.previous_twist = Twist()  # Guarda o estado anterior do cmd_vel
        self.lock = threading.Lock()

        # Inicializa a detecção da tecla de espaço
        self.key_thread = threading.Thread(target=self.detect_key)
        self.key_thread.start()

        # Flag para controlar a primeira detecção do ArUco
        self.first_detection = True

    def image_callback(self, msg: Image):
        try:
            # Converte a imagem ROS em uma imagem OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detecta os ArUcos na imagem
        marker_corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if marker_corners:
            for i in range(len(marker_corners)):
                aruco_id = ids[i][0]
                marker_corner = marker_corners[i]
                if aruco_id in self.distIDs:
                    marker_size = self.distIDs[aruco_id]
                    print(f"ID: {aruco_id}, Marker Size: {marker_size}")

                    cv_image = self.aruco_detection(marker_corner, marker_size, cv_image, aruco_id)

                    # Publica a imagem com os ArUcos detectados
                    output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    self.image_publisher.publish(output_msg)


                    # Verifica se o ArUco foi detectado pela primeira vez
                    while aruco_id == 1 and self.first_detection:
                        self.pause_movement()
        try:
            cv2.imshow("ArUco Detection", cv2.resize(cv_image, (500, 400), interpolation=cv2.INTER_AREA))
            cv2.waitKey(3)
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")

    def aruco_detection(self, marker_corner, marker_size, cv_image, aruco_id):
        _, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corner, marker_size, cam_mat, dist_coef)

        distance = np.sqrt(tVec[0][0][2] ** 2 + tVec[0][0][0] ** 2 + tVec[0][0][1] ** 2)

        aruco_pos_cam = Point()
        aruco_pos_cam.x, aruco_pos_cam.y, aruco_pos_cam.z = tVec[0][0][0], tVec[0][0][1], tVec[0][0][2]
        aruco_ori = Quaternion()
        aruco_ori.w = 1.

        # Desenha o contorno do ArUco encontrado
        cv2.polylines(cv_image, [marker_corner.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
        marker_corner = marker_corner.reshape(4, 2).astype(int)
        top_right = tuple(marker_corner[0].ravel())
        top_left = tuple(marker_corner[1].ravel())

        # Adiciona texto para distância e posições
        cv2.putText(cv_image, f"id: {aruco_id} Dist: {round(distance, 2)}", top_left, cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(cv_image, f"x: {round(tVec[0][0][0], 2)} y: {round(tVec[0][0][1], 2)} z: {round(tVec[0][0][2], 2)}", top_right, cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2, cv2.LINE_AA)

        return cv_image

    def camera_info_callback(self, msg: CameraInfo):
        # Atualiza a matriz de calibração da câmera
        self.cam_mat = np.array(msg.k).reshape((3, 3))
        self.dist_coef = np.array(msg.d)
    def pause_movement(self):
        print('Stopped')
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        self.paused = True

    def resume_movement(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.pub.publish(twist)
        self.first_detection = False
        self.paused = False

    def detect_key(self):
        def on_press(key):
            if key == keyboard.Key.space:
                with self.lock:
                    if self.paused:
                        self.resume_movement()
                    else:
                        self.pause_movement()
    
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
