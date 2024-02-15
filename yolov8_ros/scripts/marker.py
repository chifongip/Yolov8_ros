#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from visualization_msgs.msg import MarkerArray, Marker 
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes


class marker:
    def __init__(self):
        self.bbox_sub = rospy.Subscriber('yolov8/BoundingBoxes', BoundingBoxes, self.bboxCallback, queue_size=1)
        self.markerarray_pub = rospy.Publisher('yolov8/marker_array', MarkerArray, queue_size=1)
    
    def bboxCallback(self, msg):
        marker_array = MarkerArray()
        if msg.bounding_boxes:
            for bbox in msg.bounding_boxes:
                marker = Marker()
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "usb_cam_link"
                marker.ns = "maxbot"
                marker.id = bbox.id
                marker.type = 2
                marker.action = 0
                marker.pose.position.x = bbox.x
                marker.pose.position.y = bbox.y
                marker.pose.position.z = bbox.z
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = 1
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1
                marker.lifetime = rospy.Duration(3)
                marker.frame_locked = True
                marker_array.markers.append(marker)
                
            self.markerarray_pub.publish(marker_array)


def main():
    rospy.init_node('marker', anonymous=True)
    marker_cls = marker()
    rospy.loginfo("Publish Marker to Rviz.")
    rospy.spin()


if __name__ == "__main__":
    main()