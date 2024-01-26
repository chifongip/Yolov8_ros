#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math 

from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes

# P: [484.901764, 0.0, 297.242082, 0.0, 0.0, 527.320007, 193.46659, 0.0, 0.0, 0.0, 1.0, 0.0]
#     fx',        0.0, cx',        0.0, 0.0, fy',        cy',       0.0, 0.0, 0.0, 1.0, 0.0

class follower:
    def __init__(self):
        self.bbox_sub = rospy.Subscriber("/yolov8/BoundingBoxes", BoundingBoxes, self.bboxCallback, queue_size=1)
        
        P = [484.901764, 0.0, 297.242082, 0.0, 0.0, 527.320007, 193.46659, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.fx = P[0]
        self.cx = 320

        self.D_robot = 445      # mm
        
    
    def bboxCallback(self, msg):
        if msg.bounding_boxes:
            x_min = msg.bounding_boxes[0].xmin
            x_max = msg.bounding_boxes[0].xmax
            width = x_max - x_min
            
            x_c = (x_max + x_min) / 2
            x_c_pixel = x_c - self.cx

            dist_vertical = self.fx * self.D_robot / width
            dist_actual = (dist_vertical * math.sqrt(pow(self.fx, 2) + pow(x_c_pixel, 2))) / self.fx

            theta = math.degrees(math.atan(x_c_pixel / self.fx))

            print("width: ", width)
            print("dist_vertical: ", dist_vertical)
            print("dist_actual: ", dist_actual)
            print("orientation: ", theta)


def main():
    rospy.init_node('follower', anonymous=True)
    follower_cls = follower()
    rospy.loginfo("Robot Following Node Started.")
    rospy.spin()


if __name__ == "__main__":

    main()