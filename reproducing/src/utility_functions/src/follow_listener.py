#!/usr/bin/env python

# This file is a basic subscriber and publisher to direct the dead mans switch
# to the TO node

# Inputs: IOStates from UR
# Outputs: Start and stop message for to node

# Author: Robert van de Ven
# Email: robert.vandeven@wur.nl

# Import modules
import rospy

# Import message types
from ur_msgs.msg import IOStates
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from std_msgs.msg import Bool


class FollowListener(object):
    def __init__(self):
        # Create rospy node
        rospy.init_node("follow_listener")

        # Create empty global parameters
        self.DM_switch = True
        self.gripper_switch = False
        self.command = outputMsg.Robotiq2FGripper_robot_output()

        # Create subscriber for IO_states
        rospy.Subscriber("/ur_hardware_interface/io_states", IOStates, self.iofunction)

        # Create publishers
        self.gripper = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
        self.DM = rospy.Publisher('DeadMan', Bool, queue_size=1)

        # Wait for gripper control node
        rospy.wait_for_message('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input)

        # Reset the gripper
        self.command = self.genCommand('r', self.command)
        self.gripper.publish(self.command)
        rospy.sleep(2)

        # Activate the gripper
        rospy.loginfo("Activating gripper")
        self.command = self.genCommand('a', self.command)
        self.gripper.publish(self.command)
        rospy.sleep(2)

        rospy.loginfo("Ready to control")
        while not rospy.is_shutdown():
            self.listener()

    def genCommand(self, char, commando):
        """Update the command according to the character entered by the user."""

        if char == 'a':
            commando = outputMsg.Robotiq2FGripper_robot_output();
            commando.rACT = 1
            commando.rGTO = 1
            commando.rSP = 255
            commando.rFR = 150

        if char == 'r':
            commando = outputMsg.Robotiq2FGripper_robot_output();
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

    def iofunction(self, data):
        # update the IO states
        self.gripper_switch = data.digital_in_states[0].state
        self.DM_switch = data.digital_in_states[1].state

    def listener(self):
        rate = rospy.Rate(50)
        # if the DM switch is pressed:
        if not self.DM_switch:
            if not self.gripper_switch:
                # Close the gripper
                self.command = self.genCommand('c', self.command)
                # self.gripper.publish(self.command)
            else:
                # Open the gripper
                self.command = self.genCommand('o', self.command)
                # self.gripper.publish(self.command)

        # Publish the DM switch
        self.DM.publish(self.DM_switch)

        rate.sleep()


if __name__ == "__main__":
    try:
        FollowListener()
    except rospy.ROSInterruptException:
        pass
