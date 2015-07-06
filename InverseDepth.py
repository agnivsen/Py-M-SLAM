# -*- coding: utf-8 -*-
#!/usr/bin/python

import numpy as np
import math


import Param as param


def  inverseDepth(u,v,q_r,q_i,q_j,q_k,x,y,z):
    """returns the inverse depth parameters [theta, phi,rho] for the feature point specified by pixel coordinates (u,v), 
       camera pose described by unit quaternion (q_r,q_i,q_j,q_k) and camera 3D location specified by [x,y,z]
        
        Keyword arguments:
        u - pixel coordinate of the feature (u)
        v - pixel coordinate of the feature (v)
        q_r - real component of the unit quaternion
        q_i - component of unit quaternion along i-axis
        q_j - component of unit quaternion along j-axis
        q_k - component of unit quaternion along k-axis
        x - position of camera w.r.t world reference frame
        y - position of camera w.r.t world reference frame
        z - position of camera w.r.t world reference frame
        
    """
    
    q2r = get_q2r(q_r,q_i,q_j,q_k)      #generating the r_CW, for transforming the quaternion from world coordinate to camera coordinate
                    
    hw_ = np.array([[((param.Cu - u)*(param.Fu))],[((param.Cv - v)*(param.Fv))],[1]])  # there is some ambiguity with the concept of Fu and Fv. we are supposed to use (f/du) and (f/dv) here. not sure if these are same!
    hw = np.dot(q2r,hw_)
    
    inverseDepth = np.array([[math.atan(hw[0]/hw[1])],
                             [math.atan((-hw[1])/(math.sqrt((hw[0])**2 + (hw[2])**2)))],
                             [param.INITIAL_DEPTH_ESTIMATE]])
                    
    return inverseDepth
    
    
   

def get_q2r(q_r,q_i,q_j,q_k):  #this needs to be fixed
    
    x = np.array( [[(q_r**2 + q_i**2 - q_j**2 - q_k**2),((-2*q_r*q_k) + (2*q_i*q_j)),((2*q_r*q_j)+(2*q_i*q_k))],
                    [((2*q_r*q_k)+(2*q_i*q_j)),(q_r**2 - q_i**2 + q_j**2 - q_k**2),((-2*q_r*q_i) + (2*q_j*q_k))],
                    [((-2*q_r*q_j) + (2*q_i*q_k)),((2*q_r*q_i) + (2*q_j*q_k)),(q_r**2 - q_i**2 - q_j**2 + q_k**2)]])
                    
    return x
    
    
    

