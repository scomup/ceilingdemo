#!/usr/bin/python
# coding: UTF-8

import rosbag
from std_msgs.msg import Int32, String
import rospy, math, random
import numpy as np
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
import tf
from matplotlib.widgets import Button
from geometry_msgs.msg import *
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BagReader:
    def __init__(self, bagfile, image_topic, odom_topic, start_time, end_time):
        #self.scan_topic = scan_topic
        self.odom_topic = odom_topic
        self.image_topic = image_topic
        self.start_time = start_time
        self.end_time = end_time
        self.points = []
        self.odoms = []
        self.images = []
        self.data = []
        print "Bag file reading..."
        self.bag = rosbag.Bag(bagfile, 'r')
        print "image data reading..."
        self.readimage()
        print "Odom data reading..."
        self.readodom()
        print "Data sync..."
        self.syncimg()
        print "All ready."
        self.bag.close()


    def readimage(self):
        bridge = CvBridge()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.image_topic]):
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            grey_img =cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            self.images.append([time_stamp,grey_img])


    def readscan(self):
        laser_projector = LaserProjection()
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.scan_topic]):
            cloud = laser_projector.projectLaser(msg)
            frame_points = np.zeros([0,2])
            for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
                    p2d = np.array([p[0], p[1]])
                    frame_points = np.vstack([frame_points, p2d])
            self.points.append([time_stamp,frame_points])

    def readodom(self):
        for topic, msg, time_stamp in self.bag.read_messages(topics=[self.odom_topic]):
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            t = tf.transformations.quaternion_matrix((qx,qy,qz,qw))
            t[0,3] = msg.pose.pose.position.x
            t[1,3] = msg.pose.pose.position.y
            t[2,3] = msg.pose.pose.position.z
            self.odoms.append([time_stamp,t])

    def sync(self):
        idx = 0
        start_time =self.points[0][0] + rospy.Duration(self.start_time, 0)
        end_time =self.points[0][0] + rospy.Duration(self.end_time, 0)
        for time_stamp_scan,scan_data in self.points:
            if time_stamp_scan > end_time:
                    break
            if time_stamp_scan < start_time:
                continue
            time_stamp_odom,odom_data = self.odoms[idx]
            while idx < len(self.odoms) - 1:
                if time_stamp_odom > time_stamp_scan:
                    break
                time_stamp_odom,odom_data = self.odoms[idx]
                idx+=1
            self.data.append((scan_data,odom_data))

    def syncimg(self):
        idx = 0
        start_time =self.images[0][0] + rospy.Duration(self.start_time, 0)
        end_time =self.images[0][0] + rospy.Duration(self.end_time, 0)
        for time_stamp_image,image_data in self.images:
            if time_stamp_image > end_time:
                    break
            if time_stamp_image < start_time:
                continue
            time_stamp_odom,odom_data = self.odoms[idx]
            while idx < len(self.odoms) - 1:
                if time_stamp_odom > time_stamp_image:
                    break
                time_stamp_odom,odom_data = self.odoms[idx]
                idx+=1
            self.data.append((image_data,odom_data))
          



if __name__ == "__main__":
    bagreader = BagReader('ceiling_test.bag', '/stereo/left/image_raw', '/myRobot/odom',0,800)
    for data in bagreader.data:
        image, odom = data
        #cv2.imshow('image', image)
        cv2.waitKey(100)
