#!/usr/bin/env python

import os
import cv2
#from pyk4a import PyK4A
import torch
from models.common import DetectMultiBackend
from utils.dataloaders import LoadFeed
from utils.general import (check_img_size, cv2, non_max_suppression, scale_coords)
from utils.plots import Annotator, colors
import numpy as np
import rospy
from yolo_msgs.msg import yolo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_img = rospy.get_param("~view_image")
        self.augment = rospy.get_param("~augment")
        self.visualize = rospy.get_param("~visualize")
        self.hide_labels = rospy.get_param("~hide_labels")
        self.hide_conf = rospy.get_param("~hide_conf")

        # open Azure Kinect camera
        #self.k4a = PyK4A()
        #self.k4a.start()
        self.img_color = 0
        self.bridge = CvBridge()
        rospy.Subscriber("/rgb/image_raw", Image, self._update_image, queue_size=10)
        self.weights = rospy.get_param("~weights")

        # Load model
        self.device = torch.device('cuda:0')
        self.model = DetectMultiBackend(self.weights, device=self.device)  # function from original yolov5
        self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt

        # Run inference
        # capture = self.k4a.get_capture()
        # img_color = capture.color[:, :, :3]
        self.imgsz = self.img_color.shape[:2]
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size
        self.model.warmup(imgsz=(1, 3, *self.imgsz))  # warmup

        self.yolo_pub = rospy.Publisher(rospy.get_param("~output_topic"), yolo, queue_size=10)
        self.r = rospy.Rate(30)
        self.pred = yolo()
        self._run_yolo()

    def _update_image(self, img):
        frame = self.bridge.imgmsg_to_cv2(img)
        self.img_color = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)


    def detect(self):
        dataset = LoadFeed(self.img_color, img_size=self.imgsz, stride=self.stride)

        for path, im, im0s, vid_cap, s in dataset:
            im = torch.from_numpy(im).to(self.device)
            im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

            # Inference
            pred = self.model(im, augment=self.augment, visualize=self.visualize)
            # NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
            # Process predictions
            det = pred[0]  # per image
            im0 = im0s.copy()

            s += '%gx%g ' % im.shape[2:]  # print string
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(c, True))

            # Stream results
            im0 = annotator.result()
            if self.view_img:
                cv2.namedWindow('Azure Kinect', cv2.WND_PROP_FULLSCREEN) #cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                cv2.setWindowProperty('Azure Kinect',cv2.WINDOW_FULLSCREEN, cv2.WND_PROP_FULLSCREEN)
                # cv2.resizeWindow('Azure Kinect', im0.shape[1], im0.shape[0])
                cv2.imshow('Azure Kinect', im0)
                cv2.waitKey(1)  # 1 millisecond

            return det


    def rot(self, img, type):
        # convert to gray
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 0)
        maxArea = 0.0

        if type == 0: # packed
            # threshold the grayscale image
            ret, thresh = cv2.threshold(blurred, 130, 255, cv2.THRESH_BINARY_INV)

            # find outer contour
            cntrs = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]

            # savedContour = -1
            # for i in range(0, len(cntrs)):
            #     area = cv2.contourArea(cntrs[i])
            #     if area > maxArea:
            #         maxArea = area
            #         savedContour = i
        

        elif type == 1: # unpacked
            # threshold the grayscale image
            ret, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)

            # find outer contour
            cntrs = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cntrs = cntrs[0] if len(cntrs) == 2 else cntrs[1]

            # savedContour = -1
            # for i in range(0, len(cntrs)):
            #     area = cv2.contourArea(cntrs[i])
            #     if area > maxArea:
            #         maxArea = area
            #         savedContour = i

        if len(cntrs):
            join_cnts = np.concatenate(cntrs)
            rotrect = cv2.minAreaRect(join_cnts)
            # get rotated rectangle from outer contour
            # rotrect = cv2.minAreaRect(cntrs[savedContour])
            box = cv2.boxPoints(rotrect)
            box = np.intp(box)

            # draw rotated rectangle on copy of img as result
            result = img.copy()
            cv2.drawContours(result, [box], 0, (0, 0, 255), 2)


            rotrect_height = np.linalg.norm(box[0, :] - box[1, :])
            rotrect_width = np.linalg.norm(box[1, :] - box[2, :])
            blob_angle_deg = rotrect[-1]
            if (rotrect_width < rotrect_height):
                blob_angle_deg = - blob_angle_deg
            elif 0 < blob_angle_deg and blob_angle_deg < 90:
                blob_angle_deg = 90 - blob_angle_deg
            
            if blob_angle_deg == 90 or blob_angle_deg == -90 and (rotrect_width > rotrect_height):
                blob_angle_deg = 0

            blob_angle_rad = np.radians(blob_angle_deg)
            # print(blob_angle_rad, "rad")

            # cv2.imshow("RESULT", result)
            # cv2.waitKey(1)
            return -blob_angle_rad
        else:
            return -100

    def _run_yolo(self):
        while not rospy.is_shutdown():

            det = self.detect()
            det = det.cpu().detach().numpy()
            num_unpacked = np.intp(np.sum(det[:, 5]))
            num_packed = det.shape[0] - num_unpacked
            dist_packed, dist_unpacked = 100000, 100000
            saved_packed, saved_unpacked = [], []
            for c in det:
                if c[-1] == 0: # packed
                    x_center = (0.5 * (c[2] - c[0]) + c[0])
                    y_center = (0.5 * (c[3] - c[1]) + c[1])

                    current_dist = np.linalg.norm(np.array([x_center, y_center]) - np.array([640, 360]))
                    if current_dist < dist_packed:
                        dist_packed = current_dist
                        saved_packed = c
                if c[-1] == 1: # unpacked
                    x_center = (0.5 * (c[2] - c[0]) + c[0])
                    y_center = (0.5 * (c[3] - c[1]) + c[1])# up limit 360

                    current_dist = np.linalg.norm(np.array([x_center, y_center]) - np.array([640, 360]))
                    if current_dist < dist_unpacked:
                        dist_unpacked = current_dist
                        saved_unpacked = c
            

            if len(saved_packed):
                bbox = np.asarray(saved_packed, dtype=np.int32)
                x_center = (0.5 * (bbox[2] - bbox[0]) + bbox[0])
                y_center = (0.5 * (bbox[3] - bbox[1]) + bbox[1])
                tx = -((y_center - 360) * (760 / 720)) + 80  # calculate x displacement for workbench
                ty = -((x_center - 640) * (1330 / 1280)) - 40  # calculate y displacement for workbench

                cropped = self.img_color[bbox[1]:bbox[3], bbox[0]:bbox[2], :]
                rot_rad = self.rot(cropped , 0)
                self.pred.packed_tx = tx
                self.pred.packed_ty = ty
                self.pred.packed_rot = rot_rad
            else:
                self.pred.packed_tx = -100
                self.pred.packed_ty = -100
                self.pred.packed_rot = -100

            if len(saved_unpacked):
                bbox = np.asarray(saved_unpacked, dtype=np.int32)
                x_center = (0.5 * (bbox[2] - bbox[0]) + bbox[0])
                y_center = (0.5 * (bbox[3] - bbox[1]) + bbox[1])

                # need to change the calibration if the height of camera or conveyor changes
                tx = -((x_center - 640) * (540 / 1280)) -40# calculate x displacement for conveyor
                ty = ((y_center - 360) * (310 / 720)) - 80  # calculate y displacement for conveyor

                cropped = self.img_color[bbox[1]:bbox[3], bbox[0]:bbox[2], :]
                rot_rad = self.rot(cropped, 1)
                self.pred.unpacked_tx = tx
                self.pred.unpacked_ty = ty
                self.pred.unpacked_rot = rot_rad
            else:
                self.pred.unpacked_tx = -100
                self.pred.unpacked_ty = -100
                self.pred.unpacked_rot = -100


            self.yolo_pub.publish(self.pred)
            self.r.sleep()

if __name__ == "__main__":
    
    rospy.init_node("yolo_run", anonymous=True)
    detector = Yolov5Detector()
    

