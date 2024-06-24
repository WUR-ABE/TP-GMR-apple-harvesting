import numpy as np
import os
import pbdlib as pbd
import sklearn as sk
import pickle
import matplotlib.pyplot as plt
import math as m
import time as t
import sys

# Required for rel import
new_path = os.getcwd()

if new_path not in sys.path:
    sys.path.append(new_path)
from gmmr import GMMR
from hmmlqr import HMMLQR

dp = os.path.dirname(pbd.__file__) + '/data/demos/demo_2021/placing' # Path to the data
filelist = next(os.walk(dp))[2]
data = []
for f in filelist:
    temp = np.genfromtxt(dp + '/' + f, delimiter=',')
    data.append(temp)

select = range(0, len(data[0]), 20)
data_short = []
for d in data:
    data_short.append(d[select, :])

data_split_train, data_split_test = sk.model_selection.train_test_split(data_short, test_size=0.2, random_state=1337)

params_states = [15, 13, 11, 9, 7, 5]
params_hmm_u = [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0]
params_demo_count = [10, 20, 40, 60, 80, 100]

for count in params_demo_count:
    # Get right amount of demos
    data_train = data_split_train[0:count]

    # Train GMM
    gmr_model = GMMR()
    gmr_model.fit(data_train)

    # Train HMM
    lqr_model = HMMLQR()
    lqr_model.fit(data_train)

    # Pickling the results
    gmr_string = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom/models/" + "placing" + "/gmr/" + 'demos_' + str(count) + '.pickle'
    pickle.dump(gmr_model, open(gmr_string, 'wb'))

    lqr_string = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom/models/" + "placing" + "/lqr/" + 'demos_' + str(count) + '.pickle'
    pickle.dump(lqr_model, open(lqr_string, 'wb'))

data_train = data_split_train
for states in params_states:
    # Train GMM
    gmr_model = GMMR(nb_states=states)
    gmr_model.fit(data_train)

    # Train HMM
    lqr_model = HMMLQR(nb_states=states)
    lqr_model.fit(data_train)

    # Pickling the results
    gmr_string = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom/models/" + "placing" + "/gmr/" + 'states_' + str(states) + '.pickle'
    pickle.dump(gmr_model, open(gmr_string, 'wb'))

    lqr_string = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom/models/" + "placing" + "/lqr/" + 'states_' + str(states) + '.pickle'
    pickle.dump(lqr_model, open(lqr_string, 'wb'))

data_train = data_split_train
for hmm_u in params_hmm_u:
    # Train HMM
    lqr_model = HMMLQR(gmm_u=hmm_u)
    lqr_model.fit(data_train)

    # Pickling the results
    lqr_string = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom/models/" + "placing" + "/lqr/" + 'u_' + str(hmm_u) + '.pickle'
    pickle.dump(lqr_model, open(lqr_string, 'wb'))
