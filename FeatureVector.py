# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import math

import Param as param
import Transformations as trnsfrm
    
def getInitialStateVector():
    state = np.array([  [param.INIT_X],             #0
                        [param.INIT_Y],             #1
                        [param.INIT_Z],             #2
                        [param.QUAT_INIT_REAL],     #3
                        [param.QUAT_INIT_I],        #4
                        [param.QUAT_INIT_J],        #5
                        [param.QUAT_INIT_K],        #6
                        [param.INIT_V_X],           #7
                        [param.INIT_V_Y],           #8
                        [param.INIT_V_Z],           #9
                        [param.INIT_OMEGA_X],       #10
                        [param.INIT_OMEGA_Y],       #11
                        [param.INIT_OMEGA_Z]    ])  #12
                        
    return state

    
def getTransitionQuat(stateVec,del_t):
    disp_norm = math.sqrt((stateVec[0]*del_t)**2 + (stateVec[1]*del_t)**2 + (stateVec[2]*del_t)**2)
    angularAcc = np.array([[(stateVec[0]*del_t)/disp_norm],[(stateVec[1]*del_t)/disp_norm],[(stateVec[2]*del_t)/disp_norm]])
    angularAcc_norm = math.sqrt((angularAcc[0]**2) + (angularAcc[1]**2) + (angularAcc[2]**2))
    
    transition_quat = np.array([[math.cos(disp_norm/2)],     #resulting in division by zero, when no angular velocity applied
                                [(angularAcc[0]/angularAcc_norm)*math.sin(disp_norm/2)],
                                [(angularAcc[1]/angularAcc_norm)*math.sin(disp_norm/2)],
                                [(angularAcc[2]/angularAcc_norm)*math.sin(disp_norm/2)]])
                                
    return transition_quat

def stateVectorUpdate(stateVec,del_t):
    
    transition_quat = getTransitionQuat(stateVec[10:13],del_t)
    
                                
    result_quat = trnsfrm.quaternion_multiply(stateVec[3:7],transition_quat)
    
    return np.array([   [stateVec[0] + (stateVec[7]*del_t)],             #0
                        [stateVec[1] + (stateVec[8]*del_t)],             #1
                        [stateVec[2] + (stateVec[9]*del_t)],             #2
                        [result_quat[0]],     #3
                        [result_quat[1]],        #4
                        [result_quat[2]],        #5
                        [result_quat[3]],        #6
                        [stateVec[7]],           #7
                        [stateVec[8]],           #8
                        [stateVec[9]],           #9
                        [stateVec[10]],       #10
                        [stateVec[11]],       #11
                        [stateVec[12]]    ])  #12
                        



        
    

