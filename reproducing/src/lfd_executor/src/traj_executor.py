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
import lfd_executor.msg as lfd_msgs
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg


class TrajectoryActionServer(object):
    def __init__(self):
        # Create rospy node
        rospy.init_node("trajectory_executor")

        # Get params
        self.publish_rate = int(rospy.get_param('~pubish_rate', '6'))

        # Create params
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command = self.genCommand('a', self.command)

        # Create publishers
        self.gripper = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        self.pose = rospy.Publisher('/relaxed_ik/ee_pose_goals', PoseStamped, queue_size=1)

        # Set feedback and result message
        self.feedback = lfd_msgs.TrajectoryExecuteFeedback()
        self.result = lfd_msgs.TrajectoryExecuteResult()

        # Create action server
        self.server = actionlib.SimpleActionServer("action_traj_exec", lfd_msgs.TrajectoryExecuteAction,
                                                   execute_cb=self.send_command, auto_start=False)
        self.server.start()
        rospy.spin()

    def send_command(self, goal):
        # Helper variables
        r = rospy.Rate(self.publish_rate)
        success = True

        count_points = len(goal.trajectory_part_1)+len(goal.trajectory_part_2)
        rospy.loginfo("Executing trajectory of %i datapoints at a frequency of %i Hertz",
                      count_points, self.publish_rate)

        # Set up feedback
        self.feedback.current_target = PoseStamped()

        # Open the gripper
        self.command = self.genCommand('o', self.command)
        self.gripper.publish(self.command)
        rospy.sleep(0.3)

        for i in range(count_points):
            # Check the preempt from action server
            if self.server.is_preempt_requested():
                rospy.loginfo("Trajectory execution preempted")
                self.server.set_preempted()
                success = False
                break

            # Determine the current pose
            if i < len(goal.trajectory_part_1):
                # Set pose
                self.feedback.current_target.pose = goal.trajectory_part_1[i]
            elif i == len(goal.trajectory_part_1):
                # Set pose
                rospy.loginfo("Closing gripper")
                self.feedback.current_target.pose = goal.trajectory_part_1[-1]
                rospy.sleep(0.5)

                # Close the gripper
                self.command = self.genCommand('c', self.command)
                self.gripper.publish(self.command)
                rospy.sleep(0.2)

            else:
                # Second part of the trajectory
                # Reduce counter by amount from first traj
                j = i - len(goal.trajectory_part_1)
                # Set pose
                self.feedback.current_target.pose = goal.trajectory_part_2[j]

            # Set the stamp to latest and correct header
            self.feedback.current_target.header.stamp = rospy.Time()
            self.feedback.current_target.header.frame_id = "relaxed_ik"

            # Publish the current pose
            self.pose.publish(self.feedback.current_target)
            self.server.publish_feedback(self.feedback)

            r.sleep()

        # Open the gripper again
        rospy.sleep(0.5)
        self.command = self.genCommand('o', self.command)
        self.gripper.publish(self.command)

        if success:
            self.result.done = True
            self.server.set_succeeded(self.result)
            rospy.loginfo("Trajectory execution succeeded")
    
    def genCommand(self, char, commando):
        """Update the command according to the character entered by the user."""

        if char == 'a':
            commando = outputMsg.Robotiq2FGripper_robot_output()
            commando.rACT = 1
            commando.rGTO = 1
            commando.rSP = 255
            commando.rFR = 255

        if char == 'r':
            commando = outputMsg.Robotiq2FGripper_robot_output()
            commando.rACT = 0

        if char == 'c':
            commando.rPR = 255

        if char == 'o':
            commando.rPR = 0

            # If the command entered is a int, assign this value to rPRA
        try:
            commando.rPR = int(char)
            if commando.rPR > 255:
                commando.rPR = 255
            if commando.rPR < 0:
                commando.rPR = 0
        except ValueError:
            pass

        if char == 'f':
            commando.rSP += 25
            if commando.rSP > 255:
                commando.rSP = 255

        if char == 'l':
            commando.rSP -= 25
            if commando.rSP < 0:
                commando.rSP = 0

        if char == 'i':
            commando.rFR += 25
            if commando.rFR > 255:
                commando.rFR = 255

        if char == 'd':
            commando.rFR -= 25
            if commando.rFR < 0:
                commando.rFR = 0

        return commando

if __name__ == "__main__":
    TrajectoryActionServer()
