import cv2
import numpy
def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (height, width) = image.shape[:2]
    print image[300,90]
    img2 = numpy.zeros((height/2 ,width/2 ,1),numpy.uint8)
    row1 = col1 = 0
    for col in range(0,width,2):
        col1 = col1 + 1
        row1 = 0
        for row in range(0,height,2):
            row1 = row1 + 1
            black_count = 0
            white_count = 0
            grey_count  = 0
            if image[row,col] == 0 or image[row+1,col] == 0 or image[row,col+1] == 0 or image[row+1,col+1] == 0 :
                    black_count = 1
            if image[row,col] == 254 or image[row+1,col] == 254 or image[row,col+1] == 254 or image[row+1,col+1] == 254 :
                    white_count = 1
            if image[row,col] == 205 or image[row+1,col] == 205 or image[row,col+1] == 205 or image[row+1,col+1] == 205 :
                    grey_count = 1
            if black_count == 1:
                img2[row1-1,col1-1] = 0

                continue
            if white_count == 1:
                img2[row1-1,col1-1] = 254
                continue
            if grey_count == 1:
                img2[row1-1,col1-1] = 205
                
            # p = img.getPixel(col,row)
            # newImage.setPixel(col*factor,row*factor,p)

    # if both the width and height are None, then return the
    # original image
    # if width is None and height is None:
    #     return image

    # # check to see if the width is None
    # if width is None:
    #     # calculate the ratio of the height and construct the
    #     # dimensions
    #     r = height / float(h)
    #     dim = (int(w * r), height)

    # # otherwise, the height is None
    # else:
    #     # calculate the ratio of the width and construct the
    #     # dimensions
    #     r = width / float(w)
    #     dim = (width, int(h * r))

    # resize the image
    # resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return img2

src = cv2.imread('full.pgm', cv2.IMREAD_UNCHANGED)
image = image_resize(src, height = 800)
cv2.imwrite('full1.pgm',image) 
cv2.waitKey(0)
# close all open windows
cv2.destroyAllWindows()