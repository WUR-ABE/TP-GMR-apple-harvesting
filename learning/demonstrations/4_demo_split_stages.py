#!/usr/bin/env python

import os
import pandas as pd

cwd = os.getcwd()
drlist = next(os.walk(cwd))[1]

# Set the directory for the split files and remove this from the directorylist
path_1 = "approaching"
path_2 = "placing"
path_3 = "returning"
drlist.remove(path_1)
drlist.remove(path_2)
drlist.remove(path_3)

# Loop through directories in folder
for dr in drlist:
    # State the directory being read
    print("Now trimming " + dr)
    # Create list of files and remove bag file
    filelist = next(os.walk(cwd+"/"+dr))[2]
    filelist.remove(dr+".bag")
    
    # Load the ur_io csv into a DF
    ur_io_csv = cwd+"/"+dr+"/_slash_ur_hardware_interface_slash_io_states.csv"
    ur_io = pd.read_csv(ur_io_csv)
    # Find edges, where gripper was closed or opened
    edges = ur_io.pin.shift(periods=1, freq=None, axis=0)^ur_io.pin
    edges = edges.loc[edges]
    
    # Based on whether the gripper was closed before grasping or not, select right edges
    if len(edges) == 2:
        grasp_time = ur_io.loc[edges.index[0],"rosbagTimestamp"]
        release_time = ur_io.loc[edges.index[1],"rosbagTimestamp"]
    elif len(edges) == 4:
        # if dr == "trial_125":
        #     grasp_time = ur_io.loc[edges.index[2],"rosbagTimestamp"]
        #     release_time = ur_io.loc[edges.index[3],"rosbagTimestamp"]
        # else:
        grasp_time = ur_io.loc[edges.index[1],"rosbagTimestamp"]
        release_time = ur_io.loc[edges.index[2],"rosbagTimestamp"]
    elif len(edges) == 6 or len(edges) == 5:
        grasp_time = ur_io.loc[edges.index[2],"rosbagTimestamp"]
        release_time = ur_io.loc[edges.index[3],"rosbagTimestamp"]
    else:
        print("Number of edges is "+str(len(edges))+", while it should be 2, 4, 5 or 6")
        continue
    
    # Loop through all files in each directory    
    for f in filelist:
        csv = cwd+"/"+dr+"/"+f
        data = pd.read_csv(csv)
        
        # Split the file based on grasping and releasing time
        part_1 = data.loc[(data.rosbagTimestamp < grasp_time)]
        part_2 = data.loc[(data.rosbagTimestamp >= grasp_time) & (data.rosbagTimestamp <= release_time)]
        part_3 = data.loc[(data.rosbagTimestamp > release_time)]
        
        part_1.to_csv(cwd+"/"+path_1+"/"+dr+f, index = False)
        part_2.to_csv(cwd+"/"+path_2+"/"+dr+f, index = False)
        part_3.to_csv(cwd+"/"+path_3+"/"+dr+f, index = False)
