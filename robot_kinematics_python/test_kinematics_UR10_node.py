#! /usr/bin/env python
# from __future__ import print_function
import rospy, math, sys
import numpy as np
from generic_kinematics_model.kinematics import Kinematics

# Global Variable
PI = math.pi

if __name__ == '__main__':

    rospy.init_node('test_kinematics_UR10_node')    

    # ******************************************************************
    #  * Robot Parameters... these should be defined in a yaml file!
    #  *****************************************************************
    # Degress of Freedom    
    ur10_dof       = 6

    # Limits
    ur10_maxPos   = [PI, PI, PI, PI, PI, PI]
    ur10_minPos   = [-PI, -PI, -PI, -PI, -PI, -PI]
    ur10_maxVel   = [PI, PI, PI, 120.0/180.0 * PI, 120.0/180.0 * PI, 120.0/180.0 * PI]
    
    # DH Parameters
    ur10_DH_a      = [0.0 ,-0.612 ,-0.5723 , 0.0 , 0.0 , 0.0]
    ur10_DH_d      = [0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922]
    ur10_DH_alpha  = [PI/2.0, 0.0, 0.0, PI/2.0, -PI/2.0, 0.0 ]
    ur10_DH_theta0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # ******************************************************************
    #  * Initialize Robot Kinematics Class
    #  *****************************************************************
    ur10_kinematics = Kinematics(6)

    # ******************************************************************
    #  * Set DH Parameters
    #  *
    #  * @index  : index
    #  * @a, d, alpha, theta0 : D-H parameter values
    #  * @active : active joints -> 1
    #  * @min    : minimum joint limit
    #  * @max    : maximum joint limit
    #  * @maxVel : maximum joint velocity limit ( >0 )
    #  *****************************************************************
    for i in range(ur10_dof):
        ur10_kinematics.setDH(i, ur10_DH_a[i], ur10_DH_d[i], ur10_DH_alpha[i], ur10_DH_theta0[i], 1, ur10_minPos[i], ur10_maxPos[i], ur10_maxVel[i])


    # ******************************************************************
    #  * Set Base frame
    #  *****************************************************************    
    T_base_rel = np.array([  [-0.0002037,  1.0000000,  0.0000000, 0.33000],
                   [-1.0000000, -0.0002037,  0.0000000, 0.00000],
                   [0.0000000,  0.0000000,  1.0000000,  0.48600],
                   [0.0000000, 0.0000000, 0.0000000, 1.0000000]])
    
    # Necessary for UR robots!
    T_base_UR = np.array([[-1.0,  0.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0,  0.0],
                      [0.0,   0.0, 1.0,  0.0],                      
                      [0.0 ,  0.0,  0.0, 1.0]])

    # NEED to TEST THIS!
    T_base = np.dot(T_base_rel, T_base_UR)
    ur10_kinematics.setT0(T_base)

    # ******************************************************************
    #  * Set End frame
    #  *****************************************************************

    # Necessary for UR robots!
    T_ee = np.array([[0, -1, 0, 0], 
                    [0, 0, -1, 0],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]])
    ur10_kinematics.setTF(T_ee)

    # ******************************************************************
    #  *  Initialize Kinematic Chain
    #  *****************************************************************
    ur10_kinematics.readyForKinematics()


    # ******************************************************************
    #  *  Compute Forward Kinematics for a Joint State
    #  *****************************************************************
    goal       = rospy.get_param('~goal', 1)       
    if goal == 1:
        # Joint configuration for task execution
        query_joint_pos   = [PI/2.0, -(PI/2.0)*.85, 2.0/3.0 * PI, -(2.0/3.0 *PI)*1.1, -PI/2.0, PI/4.0]
    elif goal == 2:    
        # Candle Joint configuration 
        query_joint_pos   = [ PI/2, -PI/2, -0.001943, -PI, -PI/2, -0.0009473]
    elif goal == 3: 
        # Zero joint position (right side)
        query_joint_pos   = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elif goal == 4: 
        # Zero joint position (left side)
        query_joint_pos   = [ 0, -PI, 0, 0, 0, 0]

    # Convert to desired format    
    np_query_joint_pos = np.zeros((ur10_dof, 1))
    for j in range(ur10_dof):
        np_query_joint_pos[j,0] = query_joint_pos[j]
    ur10_kinematics.setJoints(np_query_joint_pos, 1)

    joint_query = ur10_kinematics.getJoints()
    rospy.loginfo('\nCurrent joint position:\n {}'.format(joint_query))
    
    query_ee_pose = ur10_kinematics.getEndTMatrix()
    rospy.loginfo('\nCurrent ee-pose pose:\n {}'.format(query_ee_pose))


    # ******************************************************************
    #  *  Compute Jacobian for a Joint State
    #  *****************************************************************
    query_Jacobian = ur10_kinematics.getEndTMatrix()