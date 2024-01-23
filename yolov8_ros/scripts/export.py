#!/usr/bin/env python
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from ultralytics import YOLO
# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

# Load a model
# model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("../weights/yolov5n.pt")  # load a pretrained model (recommended for training)

# Use the model
# model.train(data="coco128.yaml", epochs=3)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
# results = model("https://ultralytics.com/images/bus.jpg")  # predict on an image
path = model.export(format="openvino", dynamic=True, simplify=True, imgsz=(256,320))  # export the model to ONNX format