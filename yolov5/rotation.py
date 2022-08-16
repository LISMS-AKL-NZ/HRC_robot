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


    rotrect_height = np.linalg.norm(box[0, :] - box[1, :])
    rotrect_width = np.linalg.norm(box[1, :] - box[2, :])
    blob_angle_deg = rotrect[-1]
    if (rotrect_width < rotrect_height):
        blob_angle_deg = - blob_angle_deg
    elif 0 < blob_angle_deg and blob_angle_deg < 90:
        blob_angle_deg = 90 - blob_angle_deg
    elif blob_angle_deg == 0:
        blob_angle_deg = -90

    blob_angle_rad = np.radians(blob_angle_deg)
    # print(blob_angle_rad, "rad")

    cv2.imshow("RESULT", result)
    cv2.waitKey(1)
    return blob_angle_rad
