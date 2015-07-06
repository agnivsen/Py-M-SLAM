# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import FeatureVector as feat
 
import Param as param

def jacobian_prediction(stateVec,del_t):
    
    rot_vel = np.array([[del_t,0,0],[0,del_t,0],[0,0,del_t]])
    
    unit_mat = np.eye(3)
    #unit_mat[index] = 1.0
    
    transitionQuat = feat.getTransitionQuat(stateVec[10:13],del_t)
    
    quat_quat = np.array([[transitionQuat[0],-transitionQuat[1],-transitionQuat[2],-transitionQuat[3]],
                          [transitionQuat[1],transitionQuat[0],transitionQuat[3],-transitionQuat[2]],
                          [transitionQuat[2],-transitionQuat[3],transitionQuat[0],transitionQuat[1]],
                          [transitionQuat[3],transitionQuat[2],-transitionQuat[1],transitionQuat[0]]])
    
    print quat_quat


state = np.array([  [param.INIT_X],             #0
                [param.INIT_Y],                 #1
                [param.INIT_Z],                 #2
                [param.QUAT_INIT_REAL],         #3
                [param.QUAT_INIT_I],            #4
                [param.QUAT_INIT_J],            #5
                [param.QUAT_INIT_K],            #6
                [param.INIT_V_X],               #7
                [param.INIT_V_Y],               #8
                [param.INIT_V_Z],               #9
                [param.INIT_OMEGA_X],           #10
                [param.INIT_OMEGA_Y],           #11
                [param.INIT_OMEGA_Z]    ])      #12

jacobian_prediction(state,0.5)



# -*- coding: utf-8 -*-

