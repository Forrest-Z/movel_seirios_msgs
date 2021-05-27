import cv2
 
src = cv2.imread('full.pgm', cv2.IMREAD_UNCHANGED)

#percent by which the image is resized
scale_percent = 50

#calculate the 50 percent of original dimensions
width = int(src.shape[1] * scale_percent / 100)
height = int(src.shape[0] * scale_percent / 100)

# dsize
dsize = (width, height)

# resize image
output = cv2.resize(src, dsize, cv2.INTER_AREA)

cv2.imwrite('full1.pgm',output) 
# cv2.waitKey(0)
# close all open windows
# cv2.destroyAllWindows()