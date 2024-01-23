#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os
import time
from sensor_msgs.msg import Image


def image_callback(msg):
    global cv_image

    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        print(e)


def imageRecorder():
    global cv_image

    rospy.init_node('image_listener')
    rospy.Subscriber("/usb_cam/image_rect", Image, image_callback)

    if not os.path.exists('maxbot_dataset'):
        os.makedirs('maxbot_dataset')

    count = 1

    rospy.loginfo("Starting image saver node.")
    time.sleep(1)
    
    while not rospy.is_shutdown():
        filename = 'maxbot_dataset/maxbot{}.jpg'.format(count)
        cv2.imwrite(filename, cv_image)
        count += 1
        print("image saved.")
        time.sleep(1)

    rospy.spin()


if __name__ == '__main__':
    try:
        imageRecorder()
    except rospy.ROSInterruptException:
        pass
