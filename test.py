#!/usr/bin/python
# coding: UTF-8
import cv2
import numpy as np
import cPickle as pickle

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf
import time

from readbag import BagReader


__test__3dpoint__ = False
__test__readbag__ = False
img = cv2.imread('pattern_wall_resize.jpg')

grey_img =cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


class map_point:
    def __init__(self, kp, des):
        self.kp = kp
        self.loc = np.matrix([kp.pt[0]/171.4,kp.pt[1]/171.4,1.987])
        self.des = des

orb1 = cv2.ORB(nfeatures = 3000,edgeThreshold=7, nlevels = 1)
kp, des = orb1.detectAndCompute(grey_img, None)

for i in range(len(kp)):
    mp = map_point(kp[i],des[i])
    try:
        map_points.append(mp)
        mp_matrix = np.vstack([mp_matrix, mp.loc])
    except NameError:
        map_points = [mp]
        mp_matrix = mp.loc

mp_matrix = mp_matrix.transpose()
tmp = np.zeros((1, mp_matrix.shape[1]))
tmp.fill(1)
mp_matrix = np.vstack([mp_matrix, tmp])
print len(kp)

C = np.matrix([[320., 0, 320.5], [0, 320., 240.5], [0, 0, 1.]])
R = np.matrix([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.ylim(-0,480)
plt.xlim(-0,640)
plt.gca().invert_yaxis()

#
img2 = cv2.drawKeypoints(img,kp,color=(0,255,0), flags=0)
cv2.imshow('keypoint', img2)
cv2.waitKey(1)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
orb2 = cv2.ORB(nfeatures = 500,edgeThreshold=7, nlevels = 1)
bagreader = BagReader('ceiling_test.bag', '/stereo/left/image_raw', '/myRobot/odom',0,800)
for data in bagreader.data:
    image, odom = data
    kp, l_des = orb2.detectAndCompute(image, None)
    odom[0,3] +=5.1
    odom[1,3] +=1.5
    al, be, ga = tf.transformations.euler_from_matrix(odom[0:3,0:3])
    Rwc = np.matrix(odom[0:3,0:3])
    twc = np.matrix(odom[0:3,3]).transpose()
    tcw = -Rwc.transpose()*twc
    Tcw = np.matrix(np.eye(4))
    Tcw[0:3,0:3] = Rwc.transpose()
    Tcw[0:3,3] = tcw
    cv2.imshow('camera_image', image)
    try:
        r.remove()
    except:
        pass
    mp_in_robot_cood = Tcw*mp_matrix
    mp_in_robot_cood = mp_in_robot_cood[0:3,:]
    mp_in_camera_cood = C*R*mp_in_robot_cood
    x = []
    y = []
    for j in range(mp_in_camera_cood.shape[1]):
        p = mp_in_camera_cood[:,j]
        p /= p[2]
        if p[0] >= 640 or p[0] < 0:
            continue
        if p[1] >= 480 or p[1] < 0:
            continue
        d = des[j,:]
        try:
            w_des = np.vstack([w_des, d])
        except NameError:
            w_des = d
        x.append(p[0])
        y.append(p[1])
    matches = bf.match(w_des,l_des)
    w = 0
    for m in matches:
        w += 5./(m.distance + 5.)
    print w
    r = ax.scatter(x,y,s=1,c='b')
    plt.pause(0.05)
    cv2.waitKey(1)



if __test__readbag__:
    bridge = CvBridge()
    print 'reading...'
    bag = rosbag.Bag("ceiling_test.bag", 'r')
    print 'reading...OK'
    for topic, msg, time_stamp in bag.read_messages(topics=["/stereo/left/image_raw"]):
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow('image', cv_image)
        cv2.waitKey(100)
        kp, des = orb.detectAndCompute(cv_image, None)
    

if __test__3dpoint__:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = []
    y = []
    z = []
    for mp in map_points:
        x.append(mp.loc[0])
        y.append(mp.loc[1])
        z.append(mp.loc[2])

    ax.scatter(x, y, z, c='r', marker='o')
    plt.ylim(-1,10)
    plt.xlim(-1,10)
    plt.show()

