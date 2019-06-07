#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

if __name__ == '__main__':

    index = 1  # for cf1
    initialPosition = [0, 1.5, 0]  # x,y,z coordinate for this crazyflie
    cfs = CrazyflieParser(index, initialPosition)
    cf = cfs.crazyflies[0]
    time = cfs.timeHelper

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2)  # Use EKF
    cf.setParam("stabilizer/controller", 2)  # Use mellinger controller


    # cf.setParam("ring/effect", 7)

    # 3D Figure 8 -------------------------------------------

    # Eight curve maneuver 3D
    def eightPlanar_points(r, n):
        eightPlanar = []
        t = np.linspace(-(np.pi), np.pi, n)
        x = r * np.cos(t)
        y = r * np.sin(t) * np.cos(t)
        z = np.zeros(len(t))
        eightPlanar = np.c_[x, y, z]
        return eightPlanar


    def transform(theta, p, q0):  # WHAT DOES IT RETURN?
        # transforms the vector `q0` by shifting by `p` and rotating about the `x` axis of the world frame by `theta`
        q0 = np.reshape(q0,(3,1))
        q0 = np.vstack((q0,1))
        p = np.reshape(p, (3, 1))
        R = np.array([[1,0,0], [0,np.cos(theta), -np.sin(theta)], [0,np.sin(theta),np.cos(theta)]])
        T1 = np.hstack((R, p))
        T2 = np.hstack((np.zeros(3), 1))
        T = np.vstack((T1, T2))
        q1 = np.matmul(T, q0).squeeze()
        return q1[0:3]


    # Preliminaries
    n = 100  # number of points
    t = 5 / n  # [sec] time to the next point in the curve
    r = .75  # [meters]
    height = 1  # [meters]
    gotoDuration = 6.0  # [sec] time for goto tracking
    theta = np.pi / 8  # [rad] angle to til trajectory about the world frame x

    # define planar trajectory
    eight_2d = eightPlanar_points(r, n)

    # TO DO: transform the planar trajectory by applying the transformation matrix
    p = np.asarray((0, 0, height))
    eight_3d = np.zeros(eight_2d.shape)
    for i in range(n):
        planar_wp = eight_2d[i] # new point in Figure 8 curve
        eight_3d[i] = transform(theta, p ,planar_wp)  # THERE SEEMS TO BE A MISSING PARAMETER

    # takeoff to height
    cf.takeoff(targetHeight=height, duration=gotoDuration)
    time.sleep(gotoDuration)

    # NEED TO FIX! THIS ISNT TANGENT TO FIGURE 8.
    # navigate to tangent of curve
    tmp = np.add(eight_3d[0], [0, 0, 0])
    cf.goTo(goal=tmp, yaw=0, duration=gotoDuration)
    time.sleep(gotoDuration)

    for i in range(n):
        if i == 0:
            pt = np.add(eight_3d[i], [0, 0, 0])
            cf.goTo(goal=pt, yaw=0, duration=2)
            time.sleep(2)
        else:
            pt = np.add(eight_3d[i], [0, 0, 0])
            cf.cmdPosition(pos=pt, yaw=0)
            time.sleep(0.2)

    cf.land(targetHeight=0.0, duration=15.0)
    time.sleep(15.0)
