#!/usr/bin/env python

# This file listens for the command to start predicting a trajectory
# Another node is called to predict the trajectory, which sends back the trajectory
# A third node is called to execute this trajectory

# Inputs: JointStates from UR, target object pose

# Author: Robert van de Ven
# Email: robert.vandeven@wur.nl

# Import modules
import rospy
import actionlib
import tf

# Import message types
from lfd_executor.srv import lfdPred, lfdPredResponse, TrajPred
import lfd_executor.msg as lfd_msgs
from geometry_msgs.msg import Pose, PoseStamped

class LfdController(object):
    def __init__(self):
        # Create rospy node
        rospy.init_node("lfd_controller")

        # Set params
        self.object_topic = rospy.get_param('~tf_object_topic')
        self.manipulator_topic = rospy.get_param('~tf_manipulator_topic')
        self.goal_topic = rospy.get_param('~tf_goal_topic')

        # Create action client
        self.client = actionlib.SimpleActionClient("action_traj_exec", lfd_msgs.TrajectoryExecuteAction)

        # Create TF subscribers
        self.tf_listener = tf.TransformListener()

        # Wait for required services
        rospy.wait_for_service('traj_pred_gmr_service')
        rospy.wait_for_service('traj_pred_lqr_service')
        self.client.wait_for_server()

        # Create service proxies
        self.trajectory_predictor_lqr = rospy.ServiceProxy('traj_pred_lqr_service', TrajPred)
        self.trajectory_predictor_gmr = rospy.ServiceProxy('traj_pred_gmr_service', TrajPred)

        # Create service
        serv = rospy.Service('lfd_service', lfdPred, self.handle_prediction_control)
        rospy.loginfo("Ready to predict and perform trajectory using LfD.")
        rospy.spin()

    def handle_prediction_control(self, message):
        # Get transform between cube and base
        if self.tf_listener.frameExists("relaxed_ik") and self.tf_listener.frameExists(self.object_topic):
            object_pose_trans, object_pose_rot = self.tf_listener.lookupTransform("relaxed_ik", self.object_topic, rospy.Time())

        current_object_pose = Pose()
        current_object_pose.position.x = object_pose_trans[0]
        current_object_pose.position.y = object_pose_trans[1]
        current_object_pose.position.z = object_pose_trans[2]
        current_object_pose.orientation.x = object_pose_rot[0]
        current_object_pose.orientation.y = object_pose_rot[1]
        current_object_pose.orientation.z = object_pose_rot[2]
        current_object_pose.orientation.w = object_pose_rot[3]

        # Get transform between base and tcp
        if self.tf_listener.frameExists("relaxed_ik") and self.tf_listener.frameExists(self.manipulator_topic):
            robot_pose_trans, robot_pose_rot = self.tf_listener.lookupTransform("relaxed_ik", self.manipulator_topic, rospy.Time())

        current_robot_pose = Pose()
        current_robot_pose.position.x = robot_pose_trans[0]
        current_robot_pose.position.y = robot_pose_trans[1]
        current_robot_pose.position.z = robot_pose_trans[2]
        current_robot_pose.orientation.x = robot_pose_rot[0]
        current_robot_pose.orientation.y = robot_pose_rot[1]
        current_robot_pose.orientation.z = robot_pose_rot[2]
        current_robot_pose.orientation.w = robot_pose_rot[3]

        # Get transform between goal and base
        if self.tf_listener.frameExists("relaxed_ik") and self.tf_listener.frameExists(self.goal_topic):
            goal_pose_trans, goal_pose_rot = self.tf_listener.lookupTransform("relaxed_ik", self.goal_topic, rospy.Time())

        goal_object_pose = Pose()
        goal_object_pose.position.x = goal_pose_trans[0]
        goal_object_pose.position.y = goal_pose_trans[1]
        goal_object_pose.position.z = goal_pose_trans[2]
        goal_object_pose.orientation.x = goal_pose_rot[0]
        goal_object_pose.orientation.y = goal_pose_rot[1]
        goal_object_pose.orientation.z = goal_pose_rot[2]
        goal_object_pose.orientation.w = goal_pose_rot[3]

        # Call service to predict trajectory
        if message.model_type == "GMR":
            rospy.loginfo("Predicting using GMR")
            trajectory_part_1 = self.trajectory_predictor_gmr(message.model_name_1, message.horizon_1, current_robot_pose, current_object_pose).trajectory
            trajectory_part_2 = self.trajectory_predictor_gmr(message.model_name_2, message.horizon_2, trajectory_part_1[-1], goal_object_pose).trajectory
        elif message.model_type == "LQR":
            rospy.loginfo("Predicting using LQR")
            trajectory_part_1 = self.trajectory_predictor_lqr(message.model_name_1, message.horizon_1, current_robot_pose, current_object_pose).trajectory
            trajectory_part_2 = self.trajectory_predictor_lqr(message.model_name_2, message.horizon_2, trajectory_part_1[-1], goal_object_pose).trajectory
        else:
            rospy.loginfo("Incorrect model type, should be 'GMR' or 'LQR'")
            return lfdPredResponse(False)

        # Send predicted trajectories to action server
        goal = lfd_msgs.TrajectoryExecuteGoal(trajectory_part_1=trajectory_part_1, trajectory_part_2=trajectory_part_2)

        rospy.loginfo("Ready to perform trajectory, sleep for 5 seconds")
        rospy.sleep(5.0)
        self.client.send_goal(goal)
        self.client.wait_for_result()

        return lfdPredResponse(self.client.get_result().done)


if __name__ == "__main__":
    LfdController()
