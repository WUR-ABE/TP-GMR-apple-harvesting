#!/usr/bin/env python

# Script to split tf files
import pandas as pd
import os

# Set some variables
cwd = os.getcwd()
drlist = next(os.walk(cwd))[1]
# Loop through directories in folder
for d in drlist:
    # State the directory being read
    print("Now trimming " + d)
        
    # Load the DM csv into a DF
    DM_csv = cwd+"/"+d+"/_slash_DeadMan.csv"
    DM = pd.read_csv(DM_csv)
    
    # Set start and end time
    start = min(DM.rosbagTimestamp)
    end = max(DM.rosbagTimestamp)
    
    # Trim the TF file in the sub directory to start and end time 
    columns = ['rosbagTimestamp', 'transforms', '-', 'header', 'seq', 
    'stamp', 'secs', 'nsecs', 'frame_id', 'child_frame_id', 'transform', 
    'translation', 'x', 'y', 'z', 'rotation', 'x.1', 'y.1', 'z.1', 'w']
    csv = cwd+"/"+d+"/_slash_tf.csv"
    
    data = pd.read_csv(csv, names = columns, header = 0, error_bad_lines=False, warn_bad_lines=False)
    trim = data.loc[(data.rosbagTimestamp >= start) & (data.rosbagTimestamp <= end)]
    trim.to_csv(csv, index = False)
