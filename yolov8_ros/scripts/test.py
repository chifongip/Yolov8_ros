#!/usr/bin/env python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from ultralytics import YOLO
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


# Load an official or custom model
model = YOLO('/home/ubuntu/yolo_ws/src/Yolov8_ros/yolov8_ros/weights/yolov8n.pt')  # Load an official Detect model


# Perform tracking with the model
results = model.track(source=0, show=True, imgsz=320)  # Tracking with default tracker
# results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")  # Tracking with ByteTrack tracker
