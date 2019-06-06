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

    cf.takeoff(targetHeight = 0.5, duration = 3.0)
    time.sleep(3.0)

    # FILL IN YOUR CODE HERE
    # Please try both goTo and cmdPosition
   
    # 2-D line maneuver
    tmp = np.add(initialPosition, [0.0, 0.0, 1.0])    
    cf.goTo(goal=tmp, yaw=0, duration=5.0)  
    time.sleep(5.0) 

    tmp = np.add(initialPosition, [1.8, 0.0, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=10.0)
    time.sleep(10.0)

    
    tmp = np.add(initialPosition, [1.8, 0.5, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)
    
   
    tmp = np.add(initialPosition, [0.0, 0.5, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)
    
    
    tmp = np.add(initialPosition, [0.0, 0.0, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)  

    # 2-D Circle maneuver
    
    def circle_points(r, n):
        circles  = []
        zeros = np.zeros(n)
        t = np.linspace(0, 2*np.pi, n)
        x = r * np.cos(t)
        y = r * np.sin(t)
        circles  = np.c_[x, y, zeros]
        return circles 

    n = 40	# number of points 
    t = 5/n	# time to the next point in a circle
    circ = circle_points(0.45, n)
    # print(circ)

    # navigate to tangent of curve 
    tmp = np.add([0,0,0], [0,0,1])
    cf.goTo(goal=tmp, yaw=0, duration=3.0)
    time.sleep(5.0)
    print(tmp)

    for i in range(n):
        if i==0: 
            pt = np.add(circ[i], tmp)
            cf.goTo(goal=pt, yaw=0, duration=1.0)
            time.sleep(1)
        else:
            pt = np.add(circ[i], tmp)
            cf.cmdPosition(pos=pt, yaw=0)
            time.sleep(0.5)
    print(pt)
    time.sleep(5.0)

    # Eight curve maneuver
    def eight_points(r, n):
        eight  = []
        zeros = np.zeros(n)
        t = np.linspace(0, 2*np.pi, n)
        x = r * np.cos(t)
        y = r * np.sin(t) * np.cos(t)
        eight  = np.c_[x, y, zeros]
        return eight 

    n = 60	# number of points 
    t = 5/n	# time to the next point in the curve
    eight = eight_points(1, n)
    # print(eight)

    # navigate to tangent of curve 
    tmp = np.add(eight[0], [0,0,1])
    cf.goTo(goal=tmp, yaw=0, duration=3.0)
    time.sleep(5.0)

    for i in range(n):
        if i == 0: 
            pt = np.add(eight[i], [0,0,1])
            cf.goTo(goal=pt, yaw=0, duration=1.5)
            time.sleep(1.5)
        else:
            pt = np.add(eight[i], [0,0,1])
            cf.cmdPosition(pos=pt, yaw=0)
            time.sleep(0.5)
    
    time.sleep(3.0)
    print(pt)

    # to land in original position
    tmp = [0,0,1]
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)
    print(tmp)

    cf.land(targetHeight = 0.0, duration = 15.0)
    time.sleep(15.0)

    cf.takeoff(targetHeight = 1.0, duration = 3.0)
    time.sleep(3.0)

# Eight curve maneuver 3D
    def eight3d_points(r, n):
        eight_3d  = []
        t = np.linspace(-(np.pi), np.pi, n)
        x = 0.2*t
        y = r * np.cos(t)
        z = r * np.sin(t) * np.cos(t)
        eight_3d  = np.c_[x, y, z]
        return eight_3d 

    n = 100	# number of points 
    t = 5/n	# time to the next point in the curve
    eight_3d = eight3d_points(0.6, n)
    print(eight_3d)

    # navigate to tangent of curve 
    tmp = np.add(eight_3d[0], [0,0,1])
    cf.goTo(goal=tmp, yaw=0, duration=3.0)
    time.sleep(3.0)

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
