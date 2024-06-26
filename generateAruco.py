import numpy as np
import cv2
from cv2 import aruco

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
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


aruco_type = "DICT_5X5_250"

id = 1
for id in range (10):
	#arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
	arucodict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])

	print("ArUCo type '{}' with ID '{}'".format(aruco_type, id))
	tag_size = 250
	tag = np.zeros((tag_size, tag_size, 1), dtype="uint8")
	cv2.aruco.generateImageMarker(arucodict, id, tag_size, tag, 1)

	# Save the tag g
	# enerated
	tag_name = "../../Markers/" + aruco_type + "_" + str(id) + ".png"
	cv2.imwrite(tag_name, tag)
	cv2.imshow("ArUCo Tag", tag)

	cv2.waitKey(0)
cv2.destroyAllWindows()
