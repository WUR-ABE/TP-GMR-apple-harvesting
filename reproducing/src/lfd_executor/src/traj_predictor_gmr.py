#!/usr/bin/env python

# This file listens for the service to request a predicted trajectory.
# It takes in a initial and final pose and uses these to determine a trajectory and sends this back.

# Author: Robert van de Ven
# Email: robert.vandeven@wur.nl

# Import modules
import rospy
import sys
import numpy as np
import pickle
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import message types
from lfd_executor.srv import TrajPred, TrajPredResponse
from geometry_msgs.msg import Pose

# Required for rel import
new_path = "/home/agrolegion/thesis_ws/src/includes/pbdlib_custom"

if new_path not in sys.path:
    sys.path.append(new_path)
from gmmr import GMMR


def handle_trajectory_prediction(message):
    # Convert initial pose to numpy array
    x_0, y_0, z_0 = message.init_pose.position.x, message.init_pose.position.y, message.init_pose.position.z
    roll_0, pitch_0, yaw_0 = euler_from_quaternion((message.init_pose.orientation.x, message.init_pose.orientation.y,
                                                    message.init_pose.orientation.z, message.init_pose.orientation.w))
    pose_0 = np.asarray((x_0, y_0, z_0, roll_0, pitch_0, yaw_0))

    # Convert final pose to numpy array
    x_n, y_n, z_n = message.end_pose.position.x, message.end_pose.position.y, message.end_pose.position.z
    roll_n, pitch_n, yaw_n = euler_from_quaternion((message.end_pose.orientation.x, message.end_pose.orientation.y,
                                                    message.end_pose.orientation.z, message.end_pose.orientation.w))
    pose_n = np.asarray((x_n, y_n, z_n, roll_n, pitch_n, yaw_n))

    # Combine start and end poses
    start_end = np.vstack([pose_0, pose_n])

    # Load model from file location
    model = pickle.load(open(message.model_name, "rb"))

    # Predict trajectory
    pred_traj = model.predict([start_end])[0]

    # Set the response
    pose_lst = []
    for row in pred_traj:
        pose = Pose()
        pose.position.x = row[0]
        pose.position.y = row[1]
        pose.position.z = row[2]
        quat = quaternion_from_euler(row[3], row[4], row[5])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose_lst.append(pose)

    return TrajPredResponse(pose_lst)


if __name__ == "__main__":
    rospy.init_node("traj_pred_gmr_server")
    serv = rospy.Service('traj_pred_gmr_service', TrajPred, handle_trajectory_prediction)
    rospy.spin()
