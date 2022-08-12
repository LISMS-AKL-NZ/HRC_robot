from detect import run
import cv2
from pyk4a import PyK4A
import torch
from models.common import DetectMultiBackend
from utils.general import check_img_size

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
    img_depth = capture.depth
    imgsz = img_color.shape[:2]
    imgsz = check_img_size(imgsz, s=stride)  # check image size
    run(img_color[::-1], model, device, stride, imgsz, names)