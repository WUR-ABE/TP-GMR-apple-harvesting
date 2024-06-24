#!/usr/bin/env python
# This script performs dynamic time warping to make the demonstrations the same length

import numpy as np
import os
import pbdlib as pbd

# Load data and select pose information
dp = os.path.dirname(pbd.__file__) + '/data/demos/demo_apple/placing'
filelist = next(os.walk(dp))[2]
data = []
for f in filelist:
    temp = np.genfromtxt(dp + '/' + f, delimiter=',')
    data.append(temp[1:, (9, 10, 11, 13, 14, 15, 16, 17, 18, 19)])

# Sort by length
length = []
for d in data:
    length.append(d.shape[0])
lenindex = sorted(range(len(length)), key=lambda k: length[k])
data_sorted = [data[i] for i in lenindex]


def angle_corrector(a):
    if a[-1] < -2:
        a[-1] = a[-1] + 2 * np.pi
    return a

# Adjust low angle values to get smooth data
data_s_2 = []
for p in data_sorted:
    demo = np.apply_along_axis(angle_corrector, 1, p)
    data_s_2.append(demo)

# Interpolate data to get close to the same length
data_s_3 = []
for d in data_s_2:
    count = (data_s_2[-1].shape[0]) / (d.shape[0])
    if count == 1:
        interpol = d
    else:
        size = (count * (d.shape[0] - 1), d.shape[1])
        interpol = np.zeros(size)
        for i in range(d.shape[0] - 1):
            start = d[i]
            end = d[i + 1]
            diff = start - end
            diffstep = diff / count
            for j in range(count):
                interpol[count * i + j] = start - diffstep * j
    data_s_3.append(interpol)

# Apply DTW
data_1 = pbd.utils.align_trajectories(data_s_3, nb_states=5)

# write files
storepath = os.path.dirname(pbd.__file__) + '/data/demos/demo_apple/corrected_placing/'

for i in range(len(filelist)):
    np.savetxt(storepath + filelist[i], data_1[i], delimiter=',')

# Repeat process from above for approach
dp = os.path.dirname(pbd.__file__) + '/data/demos/demo_apple/approach'
filelist = next(os.walk(dp))[2]
data = []
for f in filelist:
    temp = np.genfromtxt(dp + '/' + f, delimiter=',')
    data.append(temp[1:, (9, 10, 11, 13, 14, 15, 16, 17, 18, 19)])

length = []
for d in data:
    length.append(d.shape[0])
lenindex = sorted(range(len(length)), key=lambda k: length[k])
data_sorted = [data[i] for i in lenindex]

data_s_2 = []
for p in data_sorted:
    demo = np.apply_along_axis(angle_corrector, 1, p)
    data_s_2.append(demo)

data_s_3 = []
for d in data_s_2:
    count = (data_s_2[-1].shape[0]) / (d.shape[0])
    if count == 1:
        interpol = d
    else:
        size = (count * (d.shape[0] - 1), d.shape[1])
        interpol = np.zeros(size)
        for i in range(d.shape[0] - 1):
            start = d[i]
            end = d[i + 1]
            diff = start - end
            diffstep = diff / count
            for j in range(count):
                interpol[count * i + j] = start - diffstep * j
    data_s_3.append(interpol)

data_1 = pbd.utils.align_trajectories(data_s_3, nb_states=5)

# write files
storepath = os.path.dirname(pbd.__file__) + '/data/demos/demo_apple/corrected_approach/'

for i in range(len(filelist)):
    np.savetxt(storepath + filelist[i], data_1[i], delimiter=',')
