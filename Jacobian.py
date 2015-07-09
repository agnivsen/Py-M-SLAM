# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import math
import sys
 
import Param as param
import FeatureVector as feat

def jacobian_prediction(stateVec,del_t): # this computes Ft, Gt remains to be computed
    """
    This block calculates the jacobian of the predicted state vector, as well as the process noise.
    The Jacobian calculation involved with the computation of these two matrices are dealt here, block by block
    The equation numbers mentioned in the comments can be cross-referenced with this document: www-lehre.inf.uos.de/~svalbrec/documents/master_thesis.pdf
    
    Keyword arguments:
    
        stateVec - a cloumn vector where the first 13 x 1 sub-vector describes the camera state
        del_t - time elapsed between the previous frame and the current frame
    
    """
    
    rot_vel = np.array([[del_t,0,0],[0,del_t,0],[0,0,del_t]])       # Eqn. (4.13)
    
    unit_mat = np.eye(3)
    #unit_mat[index] = 1.0
    
    transitionQuat = feat.getTransitionQuat(stateVec[10:13],del_t)
    
    q_t_q_t1 = np.array([[transitionQuat[0],-transitionQuat[1],-transitionQuat[2],-transitionQuat[3]],          # Eqn. (4.14)
                          [transitionQuat[1],transitionQuat[0],transitionQuat[3],-transitionQuat[2]],
                          [transitionQuat[2],-transitionQuat[3],transitionQuat[0],transitionQuat[1]],
                          [transitionQuat[3],transitionQuat[2],-transitionQuat[1],transitionQuat[0]]])
                          
    q_t_quat_t1 = np.array([[stateVec[3],-stateVec[4],-stateVec[5],-stateVec[6]],           # First part of Eqn. (4.15)
                              [stateVec[4],stateVec[3],-stateVec[6],stateVec[5]],
                              [stateVec[5],stateVec[6],stateVec[3],-stateVec[4]],
                              [stateVec[6],-stateVec[5],-stateVec[4],stateVec[3]]])
                             
    
    norm_omega = feat.getNorm(stateVec[10:13])          # First used in Eq. (4.16). Quite a few of the following equations re-uses this value
   
    a_t1_omega_t1_real = np.array([-(math.sin(norm_omega*del_t/2))*stateVec[10]/norm_omega*del_t/2,     # Real part of the matrix, Eq. (4.16)
                                   -(math.sin(norm_omega*del_t/2))*stateVec[11]/norm_omega*del_t/2,
                                   -(math.sin(norm_omega*del_t/2))*stateVec[12]/norm_omega*del_t/2])    
    
    main_diagonal_1 = (((math.cos(norm_omega*del_t/2))*(stateVec[10]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[10]/norm_omega)**2)))                          
    main_diagonal_2 = (((math.cos(norm_omega*del_t/2))*(stateVec[11]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[11]/norm_omega)**2)))
    main_diagonal_3 = (((math.cos(norm_omega*del_t/2))*(stateVec[12]/norm_omega)**2*(del_t/2))+(math.sin(norm_omega*del_t/2)*(1/norm_omega)*(1 - (stateVec[12]/norm_omega)**2)))   
    
    a_t1_omega_t1_md = np.array([[main_diagonal_1[0],0,0],      # Forms up the matrix represented by Eq. (4.17)
                                 [0,main_diagonal_2[0],0],
                                 [0,0,main_diagonal_3[0]]])
    
    off_diag_trignometry = ((math.cos(norm_omega*del_t/2)*del_t/2) - (1/norm_omega*(math.sin(norm_omega*del_t/2))))/(norm_omega**2)

    angular_acc_pair_1 = stateVec[11]*stateVec[12]  #YZ    
    angular_acc_pair_2 = stateVec[10]*stateVec[12]  #XZ  
    angular_acc_pair_3 = stateVec[10]*stateVec[11]  #XY  
    
    a_t1_omega_t1_od = np.array([[0,angular_acc_pair_3*off_diag_trignometry,angular_acc_pair_2*off_diag_trignometry],   # This forms the off-diagonal elements represented by Eq. (4.18)
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
    
    
    """ Now, merging everything up to arrive at Eq. (4.13) """
    Ft = np.append(column_1,column_2,axis=1)
    Ft = np.append(Ft,column_3,axis=1)
    Ft = np.append(Ft,column_4,axis=1)
    
    _Ft = np.append(column_3,column_4,axis=1)
    
    v_max_t = (param.MAX_LINEAR_VELOCITY*del_t)**2*np.eye(3)
    omega_max_t = (param.MAX_ANGULAR_VELOCITY*del_t)**2*np.eye(3)
    
    vel_max = np.append(np.append(v_max_t,np.zeros((3,3)),axis=0),np.append(np.zeros((3,3)),omega_max_t,axis=0),axis=1)
    
    _Rt = np.dot(np.dot(_Ft,vel_max),_Ft.T)    # Here goes Eq. (4.20)
    
    print _Rt
    
    return Ft, _Rt
    


def Ht_Calculator(feature_list,state):
    
    #will be filled from parameter file     
    du,dv,u0,v0,f = 0,0,0,0,0
    
    #Rcw caculator       
    """qWC = state[3:7]
    qWC_cong =np.array([qWC[0],-1*qWC[1],-1*qWC[2],-1*qWC[3]])     """
    #Rcw = q2R(qWC_cong)
    Rcw = np.ones((3,3))
    #dierectional vector 
    hC,Drho,Dir_vector = FeaturetoH_ID(feature_list,state,Rcw)
    pixel_cordinates = hCtoPoints(hC,du,dv,u0,v0,f)    
    
    htDash = Htdash(pixel_cordinates,hC,Drho,Dir_vector,Rcw,state,feature_list)
    
    return htDash
    
def hCtoPoints(hc,du,dv,u0,v0,f):
    points = []
    for h in hc:
        Uu = u0 - ((f*h[0])/(du*h[2]))
        Vu = u0 - ((f*h[1])/(du*h[2]))
        points.append([Uu,Vu])
    return points
    
def FeaturetoH_ID(feature_list,state,Rcw): 
    H = []
    Drho = []
    Dir_vector = []
    rCW = np.squeeze(state[0:3])
    for feature in feature_list:
        position = feature.get_position()
        dir_vector = feature.rho*(position - rCW)
        drho = dir_vector  + feature.m
        hCi = np.dot(Rcw , drho)
        H.append(hCi)
        Drho.append(drho)
        Dir_vector.append(dir_vector)
        
    return H,Drho,Dir_vector 

def Htdash(pixel_cordinates,hC,Drho,Dir_vector,Rcw,state,feature_list):
    
    mdim = len(feature_list)
    ndim = mdim*6 + 13
    Ht_dash = np.zeros((2*mdim,ndim))
    for i in range(0,mdim):
        b = delHtdelMu(pixel_cordinates[i],hC[i],Drho[i],Dir_vector[i],Rcw,feature_list[i],state,ndim,i)
        Ht_dash[2*i:2*i+2,:] = b
    
    return Ht_dash
    
    
def delHtdelMu(pixel_cordinate,hCi,Drho,Dir_vector,Rcw,feature,state,ndim,index):
    
    delHgt_delMu = np.zeros((2,ndim))
    dhpdhc = delhpdelhc(hCi)
    first_feature_index = 13+(index*6)
    delHgt_delMu[:,0:13] = delHgtdelXt(hCi,Drho,Rcw,pixel_cordinate,feature,state,dhpdhc)
    try:    
        a =delHgtdelfi(Rcw,feature,Dir_vector,dhpdhc)
        delHgt_delMu[:,first_feature_index:first_feature_index+6] = a
    except:
        print index
        sys.exit(0)
    return delHgt_delMu
    
def delHgtdelfi(Rcw,feature,Dir_vector,dhpdhc):
    
    delHc_delfi = np.zeros((3,6))
    delHc_delfi[:,0:3] = feature.rho*Rcw
    delHc_delfi[:,3] = np.dot(Rcw,np.array([np.cos(feature.theta)*np.cos(feature.psi),
                                            0,-1*np.sin(feature.theta)*np.cos(feature.psi)]))
    
    delHc_delfi[:,4] = np.dot(Rcw,np.array([-1*np.sin(feature.theta)*np.sin(feature.psi),
                                            -1*np.sin(feature.psi),-1*np.cos(feature.theta)*np.sin(feature.psi)]))
    delHc_delfi[:,5] = np.dot(Rcw,Dir_vector)
    
    delHp_delfi = np.dot(dhpdhc,delHc_delfi)
    
    return delHp_delfi                              
    
def delHgtdelXt(hCi,Drho,Rcw,pixel_cordinate,feature,state,dhpdhc):
    
    delHgt_delXt = np.zeros((2,13))
    
    delHgt_delXt[:,0:3] = np.dot(dhpdhc,(-1*(feature.rho)*Rcw))
    delHgt_delXt[:,3:7] = np.dot(dhpdhc,delhpdelqwc(state,feature,Drho))
    
    return delHgt_delXt
    
def delhpdelhc(hci):  

    du,dv,u0,v0,f = 0,0,0,0,0
    delhp_delhc = np.zeros((2,3))
    delhp_delhc[0,0] = -1*f/(hci[2]*du)
    delhp_delhc[1,1] = -1*f/(hci[2]*dv)
    delhp_delhc[0,2] = (-1*f*hci[0])/((hci[2]**2)*du)
    delhp_delhc[1,2] = (-1*f*hci[1])/((hci[2]**2)*du)
    
    return delhp_delhc

def delhpdelqwc(state,feature,Drho):
    qWC = np.squeeze(state[3:7])
    qWC_congR,qWC_congI,qWC_congJ,qWC_congK = qWC[0],-1*qWC[1],-1*qWC[2],-1*qWC[3]
    #delqcong_delqr,delqcong_delqi,delqcong_delqj,delqcong_delqk = (np.zeros((3,3)) for i in range(4))
    
    delqcong_delqr = 2*np.array([[qWC_congR,-1*qWC_congK,qWC_congJ],
                                [qWC_congK, qWC_congR, -1*qWC_congI],
                                [-1*qWC_congJ,qWC_congI,qWC_congR]])
    
    delqcong_delqi = 2*np.array([[qWC_congI,qWC_congJ,qWC_congK],
                                [qWC_congJ,-1*qWC_congI, -1*qWC_congR],
                                [qWC_congK,qWC_congR,-1*qWC_congI]])
    
    delqcong_delqj = 2*np.array([[-1*qWC_congJ, qWC_congI,qWC_congR],
                                [qWC_congI,qWC_congJ, qWC_congK],
                                [-1*qWC_congR,qWC_congK,-1*qWC_congJ]])

    delqcong_delqk = 2*np.array([[-1*qWC_congK,-1*qWC_congR,qWC_congI],
                                [qWC_congR,-1*qWC_congK, qWC_congJ],
                                [qWC_congI,qWC_congJ,qWC_congK]])
    
    delhc_delqcongWC = np.zeros((3,4))
    delhc_delqcongWC[:,0] = np.dot(delqcong_delqr,Drho)
    delhc_delqcongWC[:,1] = np.dot(delqcong_delqi,Drho)
    delhc_delqcongWC[:,2] = np.dot(delqcong_delqj,Drho)
    delhc_delqcongWC[:,3] = np.dot(delqcong_delqk,Drho)
    
    delqcongWC_delqWC = -1*np.eye(4)
    delqcongWC_delqWC[0,0] = 1
    
    delhp_delqwc = np.dot(delhc_delqcongWC,delqcongWC_delqWC)
    
    return delhp_delqwc


def delHpdelHc(f,du,dv,hx,hy,hz):
    
    delHp_delHc = np.zeros((2,3)) 
    
    delHp_delHc[0:0] = (-1*f)/(du*hz)
    delHp_delHc[1:1] = (-1*f)/(dv*hz)
    
    delHp_delHc[0:2] = (f*hx)/(du*(hz**2))
    delHp_delHc[1:2] = (f*hy)/(dv*(hz**2))
    
    return delHp_delHc
 
 


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

