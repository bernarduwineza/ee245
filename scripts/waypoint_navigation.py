#!/usr/bin/env python

import math
import numpy as np
from crazyflieParser import CrazyflieParser

if __name__ == '__main__':

    index = 1   # for cf1
    initialPosition = [0,0,0] # x,y,z coordinate for this crazyflie
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
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)

    
    tmp = np.add(initialPosition, [1.8, 0.5, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=2.0)
    time.sleep(5.0)
    
   
    tmp = np.add(initialPosition, [0.0, 0.5, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=5.0)
    time.sleep(5.0)
    
    
    tmp = np.add(initialPosition, [0.0, 0.0, 1.0])
    cf.goTo(goal=tmp, yaw=0, duration=2.0)
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
    n = 20	# number of points 
    t = 5/n	# time to the next point in a circle
    circ = circle_points(0.45, n)
    print(tmp)
    for i in range(n):
        pt = np.add(circ[i], tmp)
        cf.cmdPosition(pos=pt, yaw=0)
        time.sleep(0.4)
  
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

    n = 40	# number of points 
    t = 5/n	# time to the next point in the curve
    eight = eight_points(1.7, n)
    print(eight)

    for i in range(n):
        pt = np.add(eight[i], tmp)
        cf.cmdPosition(pos=pt, yaw=0)
        time.sleep(0.2)
    
    time.sleep(5.0)

    cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(5.0)
