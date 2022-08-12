from detect import run
import cv2
from pyk4a import PyK4A
import torch
from models.common import DetectMultiBackend
from utils.general import check_img_size
from rotation import rot
import numpy as np

# open Azure Kinect camera
k4a = PyK4A()
k4a.start()

# Load model
weights = 'best.pt'
device = torch.device('cuda:0')
model = DetectMultiBackend(weights, device=device)  # function from original yolov5
stride, names, pt = model.stride, model.names, model.pt

# Run inference
capture = k4a.get_capture()
img_color = capture.color[:, :, :3]
imgsz = img_color.shape[:2]
imgsz = check_img_size(imgsz, s=stride)  # check image size
model.warmup(imgsz=(1, 3, *imgsz))  # warmup

while True:
    capture = k4a.get_capture()
    img_color = capture.color[:, :, :3] # BGRA to BGR
    # img_depth = capture.depth
    # img_depth = capture.transformed_depth
    imgsz = img_color.shape[:2]
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    # maxdep = np.max(img_depth)
    # img_depth = (255/maxdep) * img_depth
    # img_depth = cv2.flip(img_depth, 1)
    # img_depth = cv2.flip(img_depth, 0)
    # img_depth = np.stack((img_depth, np.zeros_like(img_depth),np.zeros_like(img_depth)), axis=2)
    # cv2.imshow('Azure Depth', img_depth)
    # cv2.waitKey(1)  # 1 millisecond

    det = run(img_color[::-1], model, device, stride, imgsz, names)
    for c in det:
        c =c.cpu().detach().numpy()
        if c[-1] == 0:
            x_center = (0.5 * (c[2] - c[0]) + c[0])
            y_center = (0.5 * (c[3] - c[1]) + c[1])
            tx = ((y_center - 360) * (760 / 720)) + 80 # calculate x displacement for workbench
            ty = ((x_center - 640) * (1330 / 1280)) - 50 # calculate y displacement for workbench
            print('move x '+ str(tx) + ' move y ' + str(ty) + '\n')
        if c[-1] == 1:
            x_center = (0.5 * (c[2] - c[0]) + c[0])
            y_center = (0.5 * (c[3] - c[1]) + c[1])# up limit 360
            tx = ((x_center - 640) * (780 / 1280)) - 50  # calculate x displacement for conveyor
            ty = -((y_center - 360) * (440 / 720)) - 80 # calculate y displacement for conveyor
            # print('move x '+ str(tx) + ' move y ' + str(ty) + '\n')
            c = np.asarray(c, dtype=np.int32)
            img_color = cv2.flip(img_color, 1)
            img_color = cv2.flip(img_color, 0)
            cropped = img_color[c[1]:c[3], c[0]:c[2], :]
            rot(cropped)




