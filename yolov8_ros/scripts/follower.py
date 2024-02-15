#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math 

from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist

# P: [484.901764, 0.0, 297.242082, 0.0, 0.0, 527.320007, 193.46659, 0.0, 0.0, 0.0, 1.0, 0.0]
#     fx',        0.0, cx',        0.0, 0.0, fy',        cy',       0.0, 0.0, 0.0, 1.0, 0.0

class follower:
    def __init__(self):
        self.goal_z = 1.0
        self.z_scale = 0.5
        self.x_scale = 1.0
        self.max_z = 2.5
        
        P = [484.901764, 0.0, 297.242082, 0.0, 0.0, 527.320007, 193.46659, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.fx = P[0]
        self.cx = 320

        self.D_robot = 445      # mm

        self.track_lock = False
        self.track_id = None

        self.bbox_sub = rospy.Subscriber("/yolov8/BoundingBoxes", BoundingBoxes, self.bboxCallback, queue_size=1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        
    
    def bboxCallback(self, msg):
        if self.track_lock:
            track_find = False
            for bbox in msg.bounding_boxes:
                if bbox.id == self.track_id:
                    track_find = True
                    print("following maxbot id: ", self.track_id)
                    if bbox.dist > self.max_z:
                        self.cmd_pub.publish(Twist())
                    else:
                        cmd = Twist()
                        cmd.linear.x = (bbox.z - self.goal_z) * self.z_scale
                        cmd.angular.z = -bbox.x * self.x_scale
                        self.cmd_pub.publish(cmd)
            if not track_find:
                self.cmd_pub.publish(Twist())
                self.track_lock = False
                self.track_id = None
        else:
            self.cmd_pub.publish(Twist())
            if msg.bounding_boxes:
                bbox = msg.bounding_boxes
                bbox = sorted(bbox, key=lambda x: x.dist, reverse=False)
                self.track_lock = True
                self.track_id = bbox[0].id
            else:
                self.track_lock = False
                self.track_id = None
            

        # if msg.bounding_boxes:
        #     # x_min = msg.bounding_boxes[0].xmin
        #     # x_max = msg.bounding_boxes[0].xmax
        #     # width = x_max - x_min
            
        #     # x_c = (x_max + x_min) / 2
        #     # x_c_pixel = x_c - self.cx

        #     # dist_vertical = (self.fx * self.D_robot) / width
        #     # dist_actual = ((dist_vertical * math.sqrt(pow(self.fx, 2) + pow(x_c_pixel, 2))) / self.fx) / 1000

        #     # theta = math.atan(x_c_pixel / self.fx)  # radian

        #     # # print("dist_vertical: ", dist_vertical)
        #     # # print("dist_actual: ", dist_actual)
        #     # # print("orientation: ", theta)

        #     # x_actual = dist_vertical * math.tan(theta) / 1000
        #     # z_actual = dist_actual / 1000

        #     # print("x_actual:", x_actual)
        #     # print("z_actual", z_actual)

        #     # if z_actual > self.max_z:
        #     #     self.cmd_pub.publish(Twist())
        #     # else:
        #     #     cmd = Twist()
        #     #     cmd.linear.x = (z_actual - self.goal_z) * self.z_scale
        #     #     cmd.angular.z = -x_actual * self.x_scale
        #     #     self.cmd_pub.publish(cmd)


def main():
    rospy.init_node('follower', anonymous=True)
    follower_cls = follower()
    rospy.loginfo("Robot Following Node Started.")
    rospy.spin()


if __name__ == "__main__":
    main()