#!/usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import math

import actionlib

from kortex_driver.srv import *
from kortex_driver.msg import *

from std_msgs.msg import Int16, Header
from geometry_msgs.msg import PoseStamped

class ExampleWaypointActionClient:
    def __init__(self):
        try:
            rospy.init_node('example_waypoint_action_python')

            self.PACKAGE_ACTION_IDENTIFIER = 3
            self.HOME_ACTION_IDENTIFIER = 2
            self.RETRACT_ACTION_IDENTIFER = 1

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            rospy.Subscriber("/viz_pose_upper", PoseStamped, self.upper_pose_callback)
            # rospy.spinOnce()

        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def upper_pose_callback(self, data):
        self.object_x = data.pose.position.x
        self.object_y = data.pose.position.y
        self.object_z = data.pose.position.z


    def KinovaPose_callback(self, data):
        self.last_action_notif_type = None
        req = ReadActionRequest()
        
        req.input.identifier = data.data

        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            # rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
       
        return cartesianWaypoint

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True


    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        
        req.input.identifier = self.HOME_ACTION_IDENTIFIER

        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                # return True
                return self.wait_for_action_end_or_abort()


    def example_cartesian_waypoint_action(self, object_x, object_y, object_z):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()

        # Crane positions for Kinova Arm
        goal.trajectory.append(self.FillCartesianWaypoint(object_x,  object_y,  object_z, math.radians(3.3), math.radians(180), math.radians(90), 0))

        # goal.trajectory.append(self.FillCartesianWaypoint(0.311,  0.035,  0.16, math.radians(3.3), math.radians(180), math.radians(90), 0))
        # goal.trajectory.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
        # goal.trajectory.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))


        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True

    def main(self):
        success = self.is_init_success
        # rospy.Subscriber("/viz_pose_upper", PoseStamped, upper_pose_callback)

        if success:
            self.example_send_gripper_command(0.7)
            success &=self.example_clear_faults()
            success &=self.example_subscribe_to_a_robot_notification()
            success &=self.example_home_the_robot()
            success &=self.example_cartesian_waypoint_action(self.object_x+0.03, self.object_y, self.object_z)
            rospy.sleep(3.0)
            success &=self.example_cartesian_waypoint_action(self.object_x+0.03, self.object_y, self.object_z-0.1)
            self.example_send_gripper_command(0.3)
            success &=self.example_home_the_robot()

        try:
            self.GripperSubscriber = rospy.Subscriber("/KinovaAR/Pose", Int16, self.KinovaPose_callback)
            rospy.spin()
        except:
            rospy.logerr("Failed to call ROS spin")


if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
