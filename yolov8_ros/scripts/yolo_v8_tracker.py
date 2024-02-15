#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import torch
import numpy as np
from ultralytics import YOLO
import math 

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class YOLOTracker:
    def __init__(self):
        # load parameters
        weight_path = rospy.get_param('~weight_path', '')
        image_topic = rospy.get_param('~image_topic', 'usb_cam/image_rect')
        pub_topic = rospy.get_param('~pub_topic', 'yolov8/BoundingBoxes')
        self.camera_frame = rospy.get_param('~camera_frame', 'usb_cam_link')
        self.visualize = rospy.get_param('~visualize', 'false')
        self.conf = rospy.get_param('~conf', '0.6')
        self.imgsz = rospy.get_param('~imgsz', '320')

        self.fx = 484.901764
        self.cx = 320
        self.D_robot = 445

        # which device will be used
        if (rospy.get_param('~use_cpu', 'true')):
            self.device = 'cpu'
        else:
            self.device = 'cuda'

        self.model = YOLO(weight_path)
        
        if '.pt' in weight_path:
            self.model.fuse()

        self.color_image = Image()
        self.getImageStatus = False

        # image subscribe
        # self.color_sub = rospy.Subscriber(image_topic, Image, self.imageCallback, queue_size=1, buff_size=52428800)
        self.color_sub = rospy.Subscriber(image_topic, Image, self.imageCallback, queue_size=1)

        # output publishers
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher('yolov8/detection_image', Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")


    def imageCallback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model.track(
            self.color_image, 
            show=False, 
            verbose=False, 
            conf=self.conf, 
            imgsz=self.imgsz,           # imgsz=(256,320)
            tracker="botsort.yaml", 
            persist=True
            )

        self.dectShow(results, image.height, image.width)
        cv2.waitKey(3)


    def dectShow(self, results, height, width):
        self.frame = results[0].plot()
        # print(str(results[0].speed['inference']))
        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        for result in results[0].boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = np.int64(result.xyxy[0][0].item())
            boundingBox.ymin = np.int64(result.xyxy[0][1].item())
            boundingBox.xmax = np.int64(result.xyxy[0][2].item())
            boundingBox.ymax = np.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            try:
                boundingBox.id = np.int64(result.id[0].item())
            except:
                pass
            boundingBox.probability = result.conf.item()
            boundingBox.dist, boundingBox.x, boundingBox.y, boundingBox.z = self.calPose(boundingBox.xmax, boundingBox.xmin)

            self.boundingBoxes.bounding_boxes.append(boundingBox)

        self.position_pub.publish(self.boundingBoxes)
        self.publishImage(self.frame, height, width)

        if self.visualize :
            cv2.imshow('YOLOv8', self.frame)


    def calPose(self, bbox_xmax, bbox_xmin):
        width = bbox_xmax - bbox_xmin
        x_c = (bbox_xmax + bbox_xmin) / 2
        x_c_pixel = x_c - self.cx
        dist_vertical = (self.fx * self.D_robot) / width
        dist_actual = ((dist_vertical * math.sqrt(pow(self.fx, 2) + pow(x_c_pixel, 2))) / self.fx) / 1000
        theta = math.atan(x_c_pixel / self.fx)
        x_actual = dist_vertical * math.tan(theta) / 1000
        z_actual = dist_vertical / 1000

        return dist_actual, x_actual, 0, z_actual


    def publishImage(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('yolov8_ros', anonymous=True)
    yolo_tracker = YOLOTracker()
    rospy.spin()


if __name__ == "__main__":

    main()