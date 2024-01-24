#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os
import time
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

# rostopic pub /image_saver_flag std_msgs/Int32 "data: 1"

class imageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.count = 1
        self.interval = 0.5

        self.flag = 1

        self.linear_vel_x = 0
        self.angular_vel_z = 0

        rospy.Subscriber("/usb_cam/image_rect", Image, self.imageCallback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odomCallback, queue_size=1)
        rospy.Subscriber("/image_saver_flag", Int32, self.flagCallback, queue_size=1)

        rospy.loginfo("Starting image saver node.")
        time.sleep(1)
        
    
    def flagCallback(self, msg):
        self.flag = msg.data


    def odomCallback(self, msg):
        self.linear_vel_x = abs(msg.twist.twist.linear.x)
        self.angular_vel_z = abs(msg.twist.twist.angular.z)
        

    def imageCallback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg)
            filename = 'maxbot_dataset/maxbot{}.jpg'.format(self.count)
            if self.flag == 1 and (self.linear_vel_x > 0 or self.angular_vel_z > 0):
                cv2.imwrite(filename, self.cv_image)
                print("image saved:", self.count)
                self.count += 1
                time.sleep(self.interval)
            elif self.flag == 2:
                cv2.imwrite(filename, self.cv_image)
                print("image saved:", self.count)
                self.count += 1
                time.sleep(self.interval)

        except CvBridgeError as e:
            print(e)
        

if __name__ == '__main__':
    try:
        rospy.init_node('image_listener')
        if not os.path.exists('maxbot_dataset'):
            os.makedirs('maxbot_dataset')
        image_saver = imageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
