import cv2
import numpy as np

def rot(img):
    # load image as HSV and select saturation
    hh, ww, cc = img.shape

    # convert to gray
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9), 0)
    maxArea = 0.0

    # threshold the grayscale image
    ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

    # find outer contour
    cntrs = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]

    for i in range(0, len(cntrs)):
        area = cv2.contourArea(cntrs[i])
        if area > maxArea:
            maxArea = area
            savedContour = i

    # get rotated rectangle from outer contour
    rotrect = cv2.minAreaRect(cntrs[savedContour])
    box = cv2.boxPoints(rotrect)
    box = np.int0(box)

    # draw rotated rectangle on copy of img as result
    result = img.copy()
    cv2.drawContours(result, [box], 0, (0, 0, 255), 2)

    # get angle from rotated rectangle
    angle = rotrect[-1]

    # from https://www.pyimagesearch.com/2017/02/20/text-skew-correction-opencv-python/
    # the `cv2.minAreaRect` function returns values in the
    # range [-90, 0); as the rectangle rotates clockwise the
    # returned angle trends to 0 -- in this special case we
    # need to add 90 degrees to the angle
    if angle < -45:
        angle = -(90 + angle)

    # otherwise, just take the inverse of the angle to make
    # it positive
    else:
        angle = -angle

    print(angle, "deg")

    cv2.imshow("THRESH", thresh)
    cv2.imshow("RESULT", result)
    cv2.waitKey(1)