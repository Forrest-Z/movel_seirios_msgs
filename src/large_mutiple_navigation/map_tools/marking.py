# import the necessary packages
import argparse
import cv2
import numpy as np
# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
cropping = False
def click_and_crop(event, x, y, flags, param):
	# grab references to the global variables
	global refPt, cropping
	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
	# Center coordinates
	if event == cv2.EVENT_LBUTTONDBLCLK:
		# print "8"
		refPt.append((x, y))
		cropping = True
		cv2.circle(image, (x,y), 3, (255, 0, 0), -1)

	# if event == cv2.EVENT_LBUTTONDOWN:
	# 	refPt = [(x, y)]
	# 	cropping = True
	# 	# cv2.circle(image, center_coordinates, 30, (0, 0, 255), -1)
	# # check to see if the left mouse button was released
	# elif event == cv2.EVENT_LBUTTONUP:
	# 	# record the ending (x, y) coordinates and indicate that
	# 	# the cropping operation is finished
	# 	refPt.append((x, y))
	# 	cropping = False
	# 	# draw a rectangle around the region of interest
	# 	cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
	# 	cv2.imshow("image", image)

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
args = vars(ap.parse_args())
# load the image, clone it, and setup the mouse callback function
image = cv2.imread(args["image"])
clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
# keep looping until the 'q' key is pressed
while True:
	# display the image and wait for a keypress
	cv2.imshow("image", image)
	key = cv2.waitKey(1) & 0xFF
	# if the 'r' key is pressed, reset the cropping region
	if key == ord("r"):
		image = clone.copy()
	# if the 'c' key is pressed, break from the loop
	elif key == ord("c"):
		break
# if there are two reference points, then crop the region of interest
# from teh image and display it
# print len(refPt)
if len(refPt) >= 2:
	# refPt.append((refPt[0][0],refPt[1][1])) 
	# refPt.append((refPt[1][0],refPt[0][1]))
	# print refPt[0]
	# points = []
	# points.append(refPt[0])
	# points.append(refPt[2])
	# points.append(refPt[1])
	# points.append(refPt[3])
	pts = np.array(refPt)
	roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
	mask = np.zeros(clone.shape[:2], np.uint8)
	cv2.drawContours(mask, [pts], -1, (255, 255, 255), -1, cv2.LINE_AA)

	## (3)  bit-op
	dst = cv2.bitwise_and(clone, clone, mask=mask)

	## (4) add the white background
	bg = np.ones_like(clone, np.uint8)*255
	cv2.bitwise_not(bg,bg, mask=mask)
	dst2 = bg+ dst
	cv2.imwrite("hall.pgm", dst2)
	# cv2.waitKey(0)
# close all open windows
cv2.destroyAllWindows()