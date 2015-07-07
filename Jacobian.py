# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import math
 
import Param as param
import FeatureVector as feat

def jacobian_prediction(stateVec,del_t): # this computes Ft, Gt remains to be computed
    
    rot_vel = np.array([[del_t,0,0],[0,del_t,0],[0,0,del_t]])
    
    unit_mat = np.eye(3)
    #unit_mat[index] = 1.0
    
    transitionQuat = feat.getTransitionQuat(stateVec[10:13],del_t)
    
    q_t_q_t1 = np.array([[transitionQuat[0],-transitionQuat[1],-transitionQuat[2],-transitionQuat[3]],
                          [transitionQuat[1],transitionQuat[0],transitionQuat[3],-transitionQuat[2]],
                          [transitionQuat[2],-transitionQuat[3],transitionQuat[0],transitionQuat[1]],
                          [transitionQuat[3],transitionQuat[2],-transitionQuat[1],transitionQuat[0]]])
                          
    q_t_quat_t1 = np.array([[stateVec[3],-stateVec[4],-stateVec[5],-stateVec[6]],
                              [stateVec[4],stateVec[3],-stateVec[6],stateVec[5]],
                              [stateVec[5],stateVec[6],stateVec[3],-stateVec[4]],
                              [stateVec[6],-stateVec[5],-stateVec[4],stateVec[3]]])
                             
    
    norm_omega = feat.getNorm(stateVec[10:13])
    a_t1_omega_t1_real = np.array([-(math.sin(norm_omega*del_t/2))*stateVec[10]/norm_omega*del_t/2,-(math.sin(norm_omega*del_t/2))*stateVec[11]/norm_omega*del_t/2,-(math.sin(norm_omega*del_t/2))*stateVec[12]/norm_omega*del_t/2])
    
    main_diagonal_1 = (((math.cos(norm_omega*del_t/2))*(stateVec[10]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[10]/norm_omega)**2)))                          
    main_diagonal_2 = (((math.cos(norm_omega*del_t/2))*(stateVec[11]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[11]/norm_omega)**2)))
    main_diagonal_3 = (((math.cos(norm_omega*del_t/2))*(stateVec[12]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[12]/norm_omega)**2)))   
    
    a_t1_omega_t1_md = np.array([[main_diagonal_1[0],0,0],
                                 [0,main_diagonal_2[0],0],
                                 [0,0,main_diagonal_3[0]]])
    
    off_diag_trignometry = ((math.cos(norm_omega*del_t/2)*del_t/2) - (1/norm_omega*(math.sin(norm_omega*del_t/2))))/(norm_omega**2)

    angular_acc_pair_1 = stateVec[11]*stateVec[12]  #YZ    
    angular_acc_pair_2 = stateVec[10]*stateVec[12]  #XZ  
    angular_acc_pair_3 = stateVec[10]*stateVec[11]  #XY  
    
    a_t1_omega_t1_od = np.array([[0,angular_acc_pair_3*off_diag_trignometry,angular_acc_pair_2*off_diag_trignometry],
                                 [angular_acc_pair_3*off_diag_trignometry,0,angular_acc_pair_1*off_diag_trignometry],
                                 [angular_acc_pair_2*off_diag_trignometry,angular_acc_pair_1*off_diag_trignometry,0]])  
                        
    a_t1_omega_t1_imaginary = a_t1_omega_t1_md + a_t1_omega_t1_od
    
    quat_omega_t1 = np.append(a_t1_omega_t1_real.T,a_t1_omega_t1_imaginary, axis=0)    
    
    qt_omega_t1 = np.dot(np.squeeze(q_t_quat_t1,2),quat_omega_t1)
    
    column_1 = np.append(unit_mat,np.zeros((10,3)),axis=0)

    column_2 = np.append(np.zeros((3,4)),np.squeeze(q_t_q_t1,2),axis=0)
    column_2 = np.append(column_2,np.zeros((6,4)),axis=0)
    
    column_3 = np.append(rot_vel,np.zeros((4,3)),axis=0)
    column_3 = np.append(column_3,unit_mat,axis=0)    
    column_3 = np.append(column_3,np.zeros((3,3)),axis=0)
    
    column_4 = np.append(np.zeros((3,3)),qt_omega_t1,axis=0)
    column_4 = np.append(column_4,np.zeros((3,3)),axis=0)
    column_4 = np.append(column_4,unit_mat,axis=0)
    
    R = np.append(column_1,column_2,axis=1)
    R = np.append(R,column_3,axis=1)
    R = np.append(R,column_4,axis=1)
    
    _Ft = np.append(column_3,column_4,axis=1)
    
    v_max_t = (param.MAX_LINEAR_VELOCITY*del_t)**2*np.eye(3)
    omega_max_t = (param.MAX_ANGULAR_VELOCITY*del_t)**2*np.eye(3)
    
    vel_max = np.append(np.append(v_max_t,np.zeros((3,3)),axis=0),np.append(np.zeros((3,3)),omega_max_t,axis=0),axis=1)
    
    _Rt = np.dot(np.dot(_Ft,vel_max),_Ft.T)    
    
    print _Rt
    
    return R, _Rt
    


 


"""
Testing Block
"""


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

