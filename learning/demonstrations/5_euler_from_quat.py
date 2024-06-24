#!/usr/bin/env python

import os
import pandas as pd
import numpy as np

os.chdir("/media/data/OneDrive/MSc thesis/05 Experiments/apple_harvesting/demonstrations_apple_harvesting/good")
cwd = os.getcwd()
triallist = next(os.walk(cwd))[1]

# Set the directory for the split files and remove this from the directorylist
path_1 = "approaching"
path_2 = "placing"
path_3 = "returning"
triallist.remove(path_1)
triallist.remove(path_2)
triallist.remove(path_3)

drlist = [path_1, path_2, path_3]
filelist = ["_slash_joint_states.csv", "_slash_relaxed_ik_slash_ee_pose_goals.csv"]


def euler_from_quat(x, y, z, w):
    """
    Converts quaternions with components x, y, z, w into a tuple (roll, pitch, yaw)
    This uses the extrinsic ZXY euler angles
    Sticking to quaternions is recommended
    """
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return pd.Series([roll, pitch.item(), yaw])


for dr in drlist:
    for t in triallist:
        print("Now working on "+t)
        csv_robot = cwd+"/"+dr+"/"+t+filelist[0]
        csv_hand = cwd+"/"+dr+"/"+t+filelist[1]
        data_robot = pd.read_csv(csv_robot)
        data_hand = pd.read_csv(csv_hand)
        data_hand[['roll', 'pitch', 'yaw']] = data_hand.apply(lambda x: euler_from_quat(x['x.1'], x['y.1'], x['z.1'], x['w']), axis=1)
        data_robot[['roll', 'pitch', 'yaw']] = data_robot.apply(lambda x: euler_from_quat(x['q1'], x['q2'], x['q3'], x['q0']), axis=1)
        data_robot.to_csv(csv_robot, index=False)
        data_hand.to_csv(csv_hand, index=False)
