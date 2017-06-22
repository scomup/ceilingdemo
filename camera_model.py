#!/usr/bin/python
# coding: UTF-8

import cv2
import numpy as np
from math import *
import tf
import time
class Camera_model:
    def __init__(self, fx, fy, cx, cy):
        self.C = np.matrix([[fx, 0, cx], [0, fy, cy], [0, 0, 1.]])
        self.R = np.matrix([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.orb = cv2.ORB(nfeatures = 300,edgeThreshold=20, nlevels = 1)
        self.tot_time = 0

    def get_Tcw(self,pose):
        pose_matrix = tf.transformations.euler_matrix(0.0, 0.0, pose[2])
        pose_matrix[0,3] = pose[0] 
        pose_matrix[1,3] = pose[1]
        Rwc = np.matrix(pose_matrix[0:3,0:3])
        twc = np.matrix(pose_matrix[0:3,3]).transpose()
        tcw = -Rwc.transpose()*twc
        Tcw = np.matrix(np.eye(4))
        Tcw[0:3,0:3] = Rwc.transpose()
        Tcw[0:3,3] = tcw
        return Tcw

    def update_keypoint(self, image):
        self.l_key, self.l_des = self.orb.detectAndCompute(image, None)
        
    def get_matching_weight(self, image_map, image, pose):
        Tcw = self.get_Tcw(pose)

        mp_in_robot_cood = Tcw*image_map.mp_matrix
        mp_in_robot_cood = mp_in_robot_cood[0:3,:]
        mp_in_camera_cood = self.C*self.R*mp_in_robot_cood
        mp_in_camera_cood = np.array(mp_in_camera_cood)
        start = time.time()
        mp_in_camera_cood = mp_in_camera_cood/mp_in_camera_cood[2,:]
        in_flag1 = (mp_in_camera_cood[0,:] < image.shape[1]) * (mp_in_camera_cood[0,:] >= 0)
        in_flag2 = (mp_in_camera_cood[1,:] < image.shape[0]) * (mp_in_camera_cood[1,:] >= 0)
        in_flag = in_flag1 * in_flag2
        w_des = image_map.des[in_flag,:]
        mp_in_camera_cood = mp_in_camera_cood[:,in_flag]
            
        elapsed_time = time.time() - start
        self.tot_time += elapsed_time

        if w_des.shape[0] <= 40:
            return 0.

        matches = self.bf.match(w_des,self.l_des)

        w = 0
        for m in matches:
            (x1,y1)  = mp_in_camera_cood[0:2,m.queryIdx]
            (x2,y2)  = self.l_key[m.trainIdx].pt
            d = sqrt((x1-x2)**2 + (y1-y2)**2 )
            w += 1/sqrt(d)
            #w += 1/d
        """
        image_show = np.array(image)
        for m in matches:
            if m.queryIdx > len(w_key_loc):
                pass
            (x1,y1)  = mp_in_camera_cood[0:2,m.queryIdx]
            (x2,y2)  = self.l_key[m.trainIdx].pt
            qt = [x1,y1,x2,y2]
            d = sqrt((x1-x2)**2 + (y1-y2)**2 )
            if d > 200:
                continue
            cv2.circle(image_show, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
            cv2.circle(image_show, (int(x2),int(y2)), 4, (255, 0, 0), 1)
            cv2.line(image_show, (int(x1),int(y1)), (int(x2),int(y2)), (255, 0, 0), 1)
        cv2.imshow("image",image_show)
        #print "here!"
        cv2.waitKey(100)
        """
        
        #print w
        return w


        
