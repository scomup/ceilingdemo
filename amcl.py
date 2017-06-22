#!/usr/bin/python
# coding: UTF-8

from gui import ceiling_AMCL_GUI
from camera_model import Camera_model
from image_map import Image_map
from readbag import BagReader
from odom_model import Odom_Model
from particle_cloud import Particle_cloud
from math import *
import numpy as np
import tf
import time
import cv2
import matplotlib.pyplot as plt

##########################
bagfile = '/home/liu/ceiling_test.bag'
image_topic = '/stereo/left/image_raw' 
odom_topic = '/myRobot/odom'
start_time = 2
end_time = 800
##########################
particle_num = 300
init_partcle_pose = (5.2,1.45,0)
init_partcle_trans_sigma = 0.3
init_partcle_rot_sigma = 1.5
alpha_slow =  0.1
alpha_fast = 0.1
##########################
odom_aphla1 = 0.106 #Weight of rotation error resulting from rotation
odom_aphla2 = 0.430 #Weight of rotation error resulting from translation
odom_aphla3 = 0.270 #Weight of translation error resulting from translation
odom_aphla4 = 0.210 #Weight of translation error resulting from rotation
##########################
C = np.matrix([[320., 0, 320.5], [0, 320., 240.5], [0, 0, 1.]])
R = np.matrix([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
plt.ylim(-0,480)
plt.xlim(-0,640)
plt.gca().invert_yaxis()
#########################
d_thresh_ = .1
a_thresh_ = np.pi/12
##########################

class ceiling_AMCL():
    def __init__(self, gui, raw_data, image_map):
        self.gui = gui
        self.raw_data = raw_data
        self.image_map = image_map
        self.particle_cloud = Particle_cloud(particle_num, alpha_slow, alpha_fast)
        self.particle_cloud.set_init_particles(init_partcle_pose, init_partcle_trans_sigma, init_partcle_rot_sigma)
        self.odom_model = Odom_Model(odom_aphla1, odom_aphla2, odom_aphla3, odom_aphla4)
        self.camera_model = Camera_model(320.,320.,320.5,240.5)

    def matrix_to_pose(self, odom):
        al, be, ga = tf.transformations.euler_from_matrix(odom[0:3,0:3])
        return (odom[0,3],odom[1,3],ga)

    def pose_to_matrix(self, pose):
        pose_matrix = tf.transformations.euler_matrix(0.0, 0.0, pose[2])
        pose_matrix[0,3] = pose[0] 
        pose_matrix[1,3] = pose[1]
        return pose_matrix

    def checkupdate(self):
        dx = self.pre_pose[0] - self.cur_pose[0]
        dy = self.pre_pose[1] - self.cur_pose[1]
        da = self.pre_pose[2] - self.cur_pose[2]
        trans = sqrt(dx**2 + dy**2)
        update = (trans > d_thresh_) or (fabs(da) > a_thresh_)
        return update

    def gui_update(self):
            ps = [ (p[0][0]*self.image_map.resolution,p[0][1]*self.image_map.resolution , p[0][2]) for p in self.particle_cloud.particles ]
            pose = self.particle_cloud.best_p[0]
            pose_gui = [0,0,0]
            #print pose
            #pose_gui[0] = pose[0]*self.image_map.resolution 
            #pose_gui[1] = pose[1]*self.image_map.resolution
            #pose_gui[2] = pose[2]
            gui.setdata(self.image, ps, pose_gui)


    def run(self):
        self.idx = 0
        gui.state = 1
        while True:
            time.sleep(0.03)
            #print self.idx
            if self.gui.state == 1:
                self.step()
                self.idx += 1
                self.gui_update()
            if self.gui.state == 2:
                gui.state = 0
                self.step()
                self.idx += 1
                self.gui_update()

    def step(self):
        image, odom = self.raw_data[self.idx]
        self.image = image
        #cv2.imshow("image",image)
        #image, odomx = self.raw_data[0]
        odom = np.matrix(odom)
        try:
            self.last_odom = self.cur_odom
            self.cur_odom = odom
            #odom[0,3] +=5.2
            #odom[1,3] +=1.45
            last_odom_inv = np.matrix(np.linalg.inv(self.last_odom))
            odom_delta = last_odom_inv * self.cur_odom
            pose_matrix = self.pose_to_matrix(self.cur_pose)
            new_pose_matrix = pose_matrix * odom_delta
            self.cur_pose = self.matrix_to_pose(new_pose_matrix)
            if not self.checkupdate():
                return False
        except AttributeError:
            self.cur_pose = init_partcle_pose
            self.pre_pose = init_partcle_pose
            self.cur_odom = odom
            return False

        self.particle_cloud.update_by_odom_model(self.odom_model.update, self.pre_pose, self.cur_pose)
        self.camera_model.update_keypoint(image)
        start = time.time()
        self.particle_cloud.update_by_camera_model(self.camera_model.get_matching_weight,self.image_map, image)
        elapsed_time = time.time() - start
        #print ("elapsed_time:{0}".format(elapsed_time)) + "[sec]"
        self.particle_cloud.update_by_resample()
        self.pre_pose = self.cur_pose
        #print self.camera_model.tot_time
        self.camera_model.tot_time = 0.


gui = ceiling_AMCL_GUI()
gui.start()
bagreader = BagReader(bagfile, image_topic, odom_topic, start_time, end_time)
image_map = Image_map("pattern_wall_resize.jpg",171.4,1.84)
amcl = ceiling_AMCL(gui, bagreader.data, image_map)
amcl.run()

"""
gui = LSLAMGUI()
gui.start()
costmap = Prob_map(original_point, max_dist, resolution, fre_thr , occ_thr)
costmap.read_img(image_file_name)
costmap.create_likelihood()

amcl = AMCL(bagreader.data, costmap, gui)

amcl.run()
"""
