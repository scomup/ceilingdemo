#!/usr/bin/python
# coding: UTF-8

import cv2
import numpy as np


img = cv2.imread('pattern_wall_resize.jpg')

grey_img =cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


class map_point:
    def __init__(self, kp, des, resolution, ceiling_height):
        self.kp = kp
        self.loc = np.matrix([kp.pt[0]/resolution,kp.pt[1]/resolution,ceiling_height])
        self.des = des


C = np.matrix([[320., 0, 320.5], [0, 320., 240.5], [0, 0, 1.]])
R = np.matrix([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])



class Image_map:
    def __init__(self, file_name, resolution, ceiling_height, nfeatures = 3000,edgeThreshold=7, nlevels = 1):
        self.resolution = resolution
        img = cv2.imread(file_name)
        grey_img =cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        orb1 = cv2.ORB(nfeatures = nfeatures, edgeThreshold = edgeThreshold, nlevels = nlevels)
        self.kp, self.des = orb1.detectAndCompute(grey_img, None)
        for i in range(len(self.kp)):
            mp = map_point(self.kp[i],self.des[i], resolution, ceiling_height)
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
        self.mp_matrix = mp_matrix
        self.map_points = map_points




if __name__ == '__main__':
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    image_map = Image_map("pattern_wall_resize.jpg",171.4,1.84)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x = []
    y = []
    z = []
    for mp in image_map.map_points:
        x.append(mp.loc[0,0])
        y.append(mp.loc[0,1])
        z.append(mp.loc[0,2])

    ax.scatter(x, y, z, c='r', marker='o')
    plt.ylim(-1,10)
    plt.xlim(-1,10)
    plt.show()

