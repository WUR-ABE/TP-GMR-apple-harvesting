#!/usr/bin/env python

import os
import pandas as pd
import numpy as np


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
init_poses = pd.DataFrame(columns=["trial_num", "frame_id", "child_frame_id", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw"])
count = 0

for t in triallist:
    # Load data file
    csv_relaxed = cwd+"/placing/"+t+"_slash_relaxed_ik_slash_ee_pose_goals.csv"
    data_relaxed = pd.read_csv(csv_relaxed)
    # Use first pose for cube and last for goal
    cube_init_series = data_relaxed.iloc[0]
    goal_init_series = data_relaxed.iloc[-1]

    # Calc euler angles
    cube_rpy = euler_from_quat(cube_init_series.loc["x.1"], cube_init_series.loc["y.1"], cube_init_series.loc["z.1"], cube_init_series.w)
    goal_rpy = euler_from_quat(goal_init_series.loc["x.1"], goal_init_series.loc["y.1"], goal_init_series.loc["z.1"], goal_init_series.w)

    _, trial_num = t.split("_")

    # Store poses
    cube_init = [int(trial_num), "relaxed_ik", "cube_ik",
                 cube_init_series.x,
                 cube_init_series.y,
                 cube_init_series.z,
                 cube_init_series.loc["x.1"],
                 cube_init_series.loc["y.1"],
                 cube_init_series.loc["z.1"],
                 cube_init_series.w,
                 cube_rpy.loc[0],
                 cube_rpy.loc[1],
                 cube_rpy.loc[2]]
    goal_init = [int(trial_num), "relaxed_ik", "goal_ik",
                 goal_init_series.x,
                 goal_init_series.y,
                 goal_init_series.z,
                 goal_init_series.loc["x.1"],
                 goal_init_series.loc["y.1"],
                 goal_init_series.loc["z.1"],
                 goal_init_series.w,
                 goal_rpy.loc[0],
                 goal_rpy.loc[1],
                 goal_rpy.loc[2]]

    init_poses.loc[count] = cube_init
    count += 1
    init_poses.loc[count] = goal_init
    count += 1

# Write to file
init_poses.to_csv(cwd+"/init_poses.csv", index=False)
