#!/usr/bin/env python

import os
import pandas as pd

os.chdir("/media/data/OneDrive/MSc thesis/05 Experiments/apple_harvesting/bag_files")
store_dir = "/media/data/OneDrive/MSc thesis/05 Experiments/apple_harvesting/data_files"
cwd = os.getcwd()
drlist = next(os.walk(cwd))[1]

# Set the directory for the split files and remove this from the directorylist
path_1 = "approaching"
path_2 = "placing"
path_3 = "returning"

# Loop through directories in folder
for dr in drlist:
    # State the directory being read
    print("Now trimming " + dr)

    # Create directories if they don't exist
    if not os.path.isdir(store_dir+"/"+dr):
        os.mkdir(store_dir+"/"+dr)

    if not os.path.isdir(store_dir+"/"+dr+"/"+path_1):
        os.mkdir(store_dir+"/"+dr+"/"+path_1)

    if not os.path.isdir(store_dir+"/"+dr+"/"+path_2):
        os.mkdir(store_dir+"/"+dr+"/"+path_2)

    # Create list of files
    filelist = next(os.walk(cwd+"/"+dr))[2]
    #filelist.remove("_slash_rosout.csv")
    
    # Load the ur_io csv into a DF
    gripper_input_csv = cwd+"/"+dr+"/_slash_Robotiq2FGripperRobotInput.csv"
    gripper_input = pd.read_csv(gripper_input_csv)
    # Find edges, where gripper was closed or opened
    bool_gPR = gripper_input.gPR.astype(bool)
    edges = bool_gPR.shift(periods=1, freq=None, axis=0) ^ bool_gPR
    edges = edges.loc[edges]
    
    # Based on whether the gripper was closed before grasping or not, select right edges
    grasp_time = gripper_input.loc[edges.index[0], "rosbagTimestamp"]

    # Loop through all files in each directory    
    for f in filelist:
        csv = cwd+"/"+dr+"/"+f
        data = pd.read_csv(csv)
        
        # Split the file based on grasping and releasing time
        part_1 = data.loc[(data.rosbagTimestamp < grasp_time)]
        part_2 = data.loc[(data.rosbagTimestamp >= grasp_time)]
        
        part_1.to_csv(store_dir+"/"+dr+"/"+path_1+"/"+f, index=False)
        part_2.to_csv(store_dir+"/"+dr+"/"+path_2+"/"+f, index=False)
