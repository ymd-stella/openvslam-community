import cv2
import sys

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)

fileName = "marker.png"
marker_id = int(sys.argv[1])
pixels = 100
image = aruco.drawMarker(dictionary, marker_id, pixels)
cv2.imwrite(fileName, image)
cv2.waitKey(0)
