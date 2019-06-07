#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

if __name__ == '__main__':

    index = 1   # for cf1
    initialPosition = [0,1.5,0] # x,y,z coordinate for this crazyflie
    cfs = CrazyflieParser(index, initialPosition)
    cf = cfs.crazyflies[0]
    time = cfs.timeHelper

    cf.setParam("commander/enHighLevel", 1)
    cf.setParam("stabilizer/estimator", 2) # Use EKF
    cf.setParam("stabilizer/controller", 2) # Use mellinger controller
    #cf.setParam("ring/effect", 7)

    # 3D Figure 8 -------------------------------------------
    
    # Eight curve maneuver 3D
    def eightPlanar_points(r, n):
        eightPlanar  = []
        t = np.linspace(-(np.pi), np.pi, n)
        x = r * np.cos(t)
        y = r * np.sin(t) * np.cos(t)
        z = zeros(len(t))
        eightPlanar  = np.c_[x, y, z]
        return eightPlanar 

    def transform(theta,p,q0):
        T = np.


    # Preliminaries
    n = 200	                 number of points 
    t = 5/n	                # [sec] time to the next point in the curve
    r = .75                 # [meters]
    height = 0.5            # [meters]
    gotoDuration = 3.0      # [sec] time for goto tracking 

    # define trajectory
    eight_3d = eight3d_points(r, n)

    # takeoff to height
    cf.takeoff(targetHeight = height, duration = gotoDuration)
    time.sleep(gotoDuration)

    # NEED TO FIX! THIS ISNT TANGENT TO FIGURE 8.
    # navigate to tangent of curve 
    tmp = np.add(eight_3d[0], [0,0,1])
    cf.goTo(goal=tmp, yaw=0, duration=gotoDuration)
    time.sleep(gotoDuration)

    for i in range(n):
        if i == 0: 
            pt = np.add(eight_3d[i], [0,0,0])
            cf.goTo(goal=pt, yaw=0, duration=1.5)
            time.sleep(1.5)
        else:
            pt = np.add(eight_3d[i], [0,0,1])
            cf.cmdPosition(pos=pt, yaw=0)
            time.sleep(0.4)

    cf.land(targetHeight = 0.0, duration = 15.0)
    time.sleep(15.0)
