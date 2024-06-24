#!/usr/bin/env python

import os
import pandas as pd
import numpy as np
import pose_transform as ptr
import multiprocessing as mp

os.chdir("/media/data/OneDrive/MSc thesis/05 Experiments/apple_harvesting/data_files")

cwd = os.getcwd()
repro_list = next(os.walk(cwd))[1]

drlist = ["approaching", "placing"]
rename_dict = {"rosbagTimestamp": "rosbagTimestamp", "nsecs": "frame_id", "frame_id": "child_frame_id", "translation": "x", "x": "y", "y": "z",
               "rotation": "qx", "x.1": "qy", "y.1": "qz", "z.1": "qw"}


def transform_series(pdseries):
    return ptr.Transform(quat=(pdseries.qw, pdseries.qx, pdseries.qy, pdseries.qz),
                         pos=(pdseries.x, pdseries.y, pdseries.z))


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


# DF to store initial poses
init_poses = pd.DataFrame(columns=["model_type", "parameter_type", "parameter_value", "repetition", "frame_id",
                                   "child_frame_id", "x", "y", "z", "qx", "qy", "qz", "qw", "roll", "pitch", "yaw"])
count = 0

# Good repro for testing:
repro = repro_list[32]

for repro in repro_list:

    # Cube and goal pose only determined at one point -> static transforms
    tf_csv = cwd + "/" + repro + "/_slash_tf.csv"
    df_tf = pd.read_csv(tf_csv)
    # Get the moment the action server was called
    init_timestamp = min(pd.read_csv(cwd + "/" + repro + "/approaching/_slash_action_traj_exec_slash_feedback.csv").rosbagTimestamp) - 5E9

    # Select TF messages from before this timestamp
    df_tf_init = df_tf.loc[(df_tf.rosbagTimestamp <= init_timestamp)]

    # Get the transforms between each relevant frame
    try:
        tf_base_opti_corr = df_tf_init[(df_tf_init['nsecs'].str.contains("base")) & (df_tf_init['frame_id'].str.contains("opti_corrected"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
    except IndexError:
        print "TF 'base' to 'opti_corrected' does not exist in reproduction", repro
    try:
        tf_opti_corr_exp_Nick = df_tf_init[(df_tf_init['nsecs'].str.contains("opti_corrected")) & (df_tf_init['frame_id'].str.contains("Experiment_Nick"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
        print (init_timestamp - tf_opti_corr_exp_Nick.rosbagTimestamp)/1E9
    except IndexError:
        print "TF 'opti_corrected' to 'Experiment_Nick' does not exist in reproduction", repro
    try:
        tf_exp_Nick_goal_ik = df_tf_init[(df_tf_init['nsecs'].str.contains("Experiment_Nick")) & (df_tf_init['frame_id'].str.contains("goal_ik"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
    except IndexError:
        print "TF 'Experiment_Nick' to 'goal_ik' does not exist in reproduction", repro
    try:
        tf_opti_corr_opti_cube = df_tf_init[(df_tf_init['nsecs'].str.contains("opti_corrected")) & (df_tf_init['frame_id'].str.contains("opti_cube"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
    except IndexError:
        print "TF 'opti_corrected' to 'opti_cube' does not exist in reproduction", repro
    try:
        tf_opti_cube_cube_ik = df_tf_init[(df_tf_init['nsecs'].str.contains("opti_cube")) & (df_tf_init['frame_id'].str.contains("cube_ik"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
    except IndexError:
        print "TF 'opti_cube' to 'cube_ik' does not exist in reproduction", repro
    try:
        tf_base_relaxed_ik = df_tf_init[(df_tf_init['nsecs'].str.contains("base")) & (df_tf_init['frame_id'].str.contains("relaxed_ik"))]\
            .iloc[-1].iloc[[0, 7, 8, 11, 12, 13, 15, 16, 17, 18]].rename(rename_dict)
    except IndexError:
        print "TF 'base' to 'relaxed_ik' does not exist in reproduction", repro

    matrix_base_opti_corr = transform_series(tf_base_opti_corr)
    matrix_opti_corr_exp_Nick = transform_series(tf_opti_corr_exp_Nick)
    matrix_exp_Nick_goal_ik = transform_series(tf_exp_Nick_goal_ik)
    matrix_opti_corr_opti_cube = transform_series(tf_opti_corr_opti_cube)
    matrix_opti_cube_cube_ik = transform_series(tf_opti_cube_cube_ik)
    matrix_base_relaxed_ik = transform_series(tf_base_relaxed_ik)
    matrix_relaxed_ik_base = transform_series(tf_base_relaxed_ik).inverse()

    matrix_cube_in_relaxed = ((matrix_relaxed_ik_base * matrix_base_opti_corr) * matrix_opti_corr_opti_cube) * matrix_opti_cube_cube_ik
    matrix_goal_in_relaxed = ((matrix_relaxed_ik_base * matrix_base_opti_corr) * matrix_opti_corr_exp_Nick) * matrix_exp_Nick_goal_ik

    cube_rpy = euler_from_quat(matrix_cube_in_relaxed.quaternion().x,
                               matrix_cube_in_relaxed.quaternion().y,
                               matrix_cube_in_relaxed.quaternion().z,
                               matrix_cube_in_relaxed.quaternion().w)
    goal_rpy = euler_from_quat(matrix_goal_in_relaxed.quaternion().x,
                               matrix_goal_in_relaxed.quaternion().y,
                               matrix_goal_in_relaxed.quaternion().z,
                               matrix_goal_in_relaxed.quaternion().w)

    # Use file name to obtain repro info and store in init_pose df
    model_name, param, rep = repro.split("_")

    param_type, param_value = param.split("-")
    if param_value.startswith("neg"):
        param_value = "-"+param_value[-1]
    elif param_value.startswith("pos"):
        param_value = param_value[-1]

    _, rep_num = rep.split("-")

    tf_cube_ik_relaxed_ik = [model_name, param_type, param_value, rep_num,
                             "relaxed_ik", "cube_ik",
                             matrix_cube_in_relaxed.position()[0],
                             matrix_cube_in_relaxed.position()[1],
                             matrix_cube_in_relaxed.position()[2],
                             matrix_cube_in_relaxed.quaternion().x,
                             matrix_cube_in_relaxed.quaternion().y,
                             matrix_cube_in_relaxed.quaternion().z,
                             matrix_cube_in_relaxed.quaternion().w,
                             cube_rpy.loc[0],
                             cube_rpy.loc[1],
                             cube_rpy.loc[2]]
    tf_goal_ik_relaxed_ik = [model_name, param_type, param_value, rep_num,
                             "relaxed_ik", "goal_ik",
                             matrix_goal_in_relaxed.position()[0],
                             matrix_goal_in_relaxed.position()[1],
                             matrix_goal_in_relaxed.position()[2],
                             matrix_goal_in_relaxed.quaternion().x,
                             matrix_goal_in_relaxed.quaternion().y,
                             matrix_goal_in_relaxed.quaternion().z,
                             matrix_goal_in_relaxed.quaternion().w,
                             goal_rpy.loc[0],
                             goal_rpy.loc[1],
                             goal_rpy.loc[2]]

    init_poses.loc[count] = tf_cube_ik_relaxed_ik
    count += 1
    init_poses.loc[count] = tf_goal_ik_relaxed_ik
    count += 1

init_poses.to_csv(cwd+"/init_poses.csv", index=False)


#### Part to convert joint state pose from frame "base" to "relaxed_ik"
# Set static transform
tf_relaxed_base = ptr.Transform(pos=(-0.11235, 0.29515, 0.4809), quat=(0.0, 0.0, 0.0, -1.0)).inverse()
tf_joints_tool_ik = ptr.Transform(pos=(0, 0, 0), quat=(0, 0, 0.707, -0.707))


def pose_processor(repro):
    for part in ["approaching", "placing"]:
        df_joints = pd.read_csv(cwd + "/" + repro + "/" + part + "/_slash_joint_states.csv")

        joint_poses = pd.DataFrame(columns=["rosbagTimestamp", "frame_id", "child_frame_id", "x", "y", "z", "qx", "qy", "qz",
                                            "qw", "roll", "pitch", "yaw"])
        count = 0

        for i, row in df_joints.iterrows():
            matrix_base_joint = ptr.Transform(pos=(row.position_x, row.position_y, row.position_z), quat=(row.q0, row.q1, row.q2, row.q3))
            matrix_relaxed_tool_ik = (tf_relaxed_base * matrix_base_joint) * tf_joints_tool_ik

            tool_rpy = euler_from_quat(matrix_relaxed_tool_ik.quaternion().x, matrix_relaxed_tool_ik.quaternion().y,
                                       matrix_relaxed_tool_ik.quaternion().z, matrix_relaxed_tool_ik.quaternion().w)

            row_joint_poses = [row.rosbagTimestamp, "relaxed_ik", "tool_ik", matrix_relaxed_tool_ik.position()[0],
                               matrix_relaxed_tool_ik.position()[1], matrix_relaxed_tool_ik.position()[2],
                               matrix_relaxed_tool_ik.quaternion().x, matrix_relaxed_tool_ik.quaternion().y,
                               matrix_relaxed_tool_ik.quaternion().z, matrix_relaxed_tool_ik.quaternion().w,
                               tool_rpy.loc[0], tool_rpy.loc[1], tool_rpy.loc[2]]

            joint_poses.loc[count] = row_joint_poses
            count += 1

        joint_poses.to_csv(cwd + "/" + repro + "/" + part + "/_slash_joint_pose.csv")


pool = mp.Pool()
pool.map(pose_processor, repro_list)
