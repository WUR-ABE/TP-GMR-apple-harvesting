#!/usr/bin/env python

import math as m
import numpy as np
import os
import pyquaternion as pq
import pandas as pd

# Set some variables
cwd = os.getcwd()
drlist = next(os.walk(cwd))[1]
tf_columns = ['rosbagTimestamp', 'transforms', '-', 'header', 'seq', 
    'stamp', 'secs', 'nsecs', 'frame_id', 'child_frame_id', 'transform', 
    'translation', 'x', 'y', 'z', 'rotation', 'x.1', 'y.1', 'z.1', 'w']

# Forward kinematics solver
d = np.array([0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819]) # unit: mm
a = np.array([0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0]) # unit: mm
alpha = np.array([m.pi/2, 0.0, 0.0, m.pi/2, -m.pi/2, 0.0]) # unit: radian

def HTM(i, theta):
    """Calculate the HTM between two links.
    Args:
        i: A target index of joint value. 
        theta: A list of joint value solution. (unit: radian)
    Returns:
        An HTM of Link l w.r.t. Link l-1, where l = i + 1.
    """

    Rot_z = np.matrix(np.identity(4))
    Rot_z[0, 0] = Rot_z[1, 1] = m.cos(theta[i])
    Rot_z[0, 1] = -m.sin(theta[i])
    Rot_z[1, 0] = m.sin(theta[i])

    Trans_z = np.matrix(np.identity(4))
    Trans_z[2, 3] = d[i]

    Trans_x = np.matrix(np.identity(4))
    Trans_x[0, 3] = a[i]

    Rot_x = np.matrix(np.identity(4))
    Rot_x[1, 1] = Rot_x[2, 2] = m.cos(alpha[i])
    Rot_x[1, 2] = -m.sin(alpha[i])
    Rot_x[2, 1] = m.sin(alpha[i])

    A_i = Rot_z * Trans_z * Trans_x * Rot_x
	    
    return A_i

def fwd_kin(theta, i_unit='r', o_unit='p'):
    """Solve the HTM based on a list of joint values.
    Args:
        theta: A list of joint values. (unit: radian)
        i_unit: Output format. 'r' for radian; 'd' for degree.
        o_unit: Output format. 'n' for np.array; 'p' for ROS Pose.
    Returns:
        The HTM of end-effector joint w.r.t. base joint
    """

    T_06 = np.matrix(np.identity(4))

    if i_unit == 'd':
        theta = [m.radians(i) for i in theta]
    
    for i in range(6):
        T_06 *= HTM(i, theta)

    if o_unit == 'n':
        return T_06
    elif o_unit == 'p':
        position_x = T_06[0,3]
        position_y = T_06[1,3]
        position_z = T_06[2,3]
        q = pq.Quaternion(matrix = T_06)
        
        
        return pd.Series([position_x, position_y, position_z, q[0], q[1], q[2], q[3]])

# Loop through directories in folder
for dr in drlist:
    # State the directory being read
    print("Now trimming " + dr)
    # Create list of files and remove bag file
    filelist = next(os.walk(cwd+"/"+dr))[2]
    filelist.remove(dr+".bag")
    # Load the DM csv into a DF
    DM_csv = cwd+"/"+dr+"/_slash_DeadMan.csv"
    DM = pd.read_csv(DM_csv)
    # Filter to time where DM is pressed and set start and end time
    DM = DM.loc[~DM.data]
    start = min(DM.rosbagTimestamp)
    end = max(DM.rosbagTimestamp)
    # Trim all csv files in the sub directory to start and end time 
    for f in filelist:
        csv = cwd+"/"+dr+"/"+f
        # Due to the way the robot arm is translated into TF, a special treatment is required. 
        # This removes the pose of the robot links from TF, 
        # however this data is still available by using the joint states
        if f == "_slash_tf.csv":
            data = pd.read_csv(csv, names = tf_columns, header = 0, error_bad_lines=False, warn_bad_lines=False)
        else:
            data = pd.read_csv(csv)
        
        trim = data.loc[(data.rosbagTimestamp >= start) & (data.rosbagTimestamp <= end)]
        
        # Some topics require cleaning, as converting to CSV did not go completely correctly
        if f == "_slash_joint_group_pos_controller_slash_command.csv":
            # Joint controller data has all joint data in one column,this is split into seperate columns for each joint
            trim[['shoulder_lift_joint', 'shoulder_pan_joint', 'elbow_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']] = trim.data.map(lambda x: x.lstrip('[').rstrip(']')) \
                  .str.split(", ", expand = True).apply(pd.to_numeric)
            # Drop the columns which the data was taken from
            trim = trim.drop(labels=["data"], axis=1)
            
        elif f == "_slash_joint_states.csv":
            # Same problem with joint states, except this also has joint velocity and different column names
            trim[['elbow_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']] = trim["wrist_3_joint]"].map(lambda x:x.lstrip('[').rstrip(']')) \
                  .str.split(", ", expand = True).apply(pd.to_numeric)
            trim[['elbow_joint_vel', 'shoulder_pan_joint_vel', 'shoulder_lift_joint_vel', 
                  'wrist_1_joint_vel', 'wrist_2_joint_vel', 'wrist_3_joint_vel']] = trim["position"].map(lambda x:x.lstrip('[').rstrip(']')) \
                  .str.split(", ", expand = True).apply(pd.to_numeric)
            # Based on the joint states the ee pose is also calculated
            # As forward kinematics take some time, this is printed
            print("Now calculating forward kinematics of " + dr)
            trim[['position_x', 'position_y', 'position_z', 'q0', 'q1', 'q2', 'q3']] = trim.apply(lambda x: 
                fwd_kin([x[['shoulder_lift_joint']], x[['shoulder_pan_joint']], x[['elbow_joint']], 
                         x[['wrist_1_joint']], x[['wrist_2_joint']], x[['wrist_3_joint']]]), axis = 1)
            # Drop the columns which the data was taken from
            trim = trim.drop(labels=["wrist_3_joint]","position"], axis=1)
        trim.to_csv(csv, index = False)
