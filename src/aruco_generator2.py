import numpy as np
import cv2
import os

base_dict =  os.path.expanduser('~/lognav_ws/src/freedom_vehicle/models/arucos/materials/textures/')

ARUCO_DICT = {
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
            "DICT_7X7_50": cv2.aruco.DICT_6X6_50,
            "DICT_7X7_100": cv2.aruco.DICT_6X6_100,
            "DICT_7X7_250": cv2.aruco.DICT_6X6_250,
            "DICT_7X7_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
            }
aruco_type = "DICT_4X4_1000"
id = 0
tag_size = 1000
border_size = 200
total_image = tag_size +2*border_size

for i in range (id, tag_size):
    arucoGet = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    tag = np.ones((total_image, total_image, 1), dtype="uint8") * 255
    marker_area = np.zeros((tag_size, tag_size, 1), dtype="uint8")
    cv2.aruco.drawMarker(arucoGet, id, tag_size, marker_area, 1)

    bordered_tag = tag.copy()
    start_row = border_size
    end_row = start_row + 1000
    start_col = border_size
    end_col = start_col + 1000
    bordered_tag[start_row:end_row, start_col:end_col] = marker_area
    tag_name = os.path.join(base_dict) + str(id) + '.png'
    print("asfas", tag_name)
    cv2.imwrite(tag_name, bordered_tag)
    id += 1

cv2.waitKey(0)
cv2.destroyAllWindows()