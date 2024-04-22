#!/usr/bin/env python3

import rospy
from realsense_segment2.srv import AddTwoInts

def add_two_numbers_client(num1, num2):
    rospy.wait_for_service('add_two_numbers')
    try:
        add_numbers = rospy.ServiceProxy('add_two_numbers', AddTwoInts)
        response = add_numbers(num1, num2)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('add_two_numbers_client')
    num1 = float(raw_input("Enter first number: "))
    num2 = float(raw_input("Enter second number: "))
    result = add_two_numbers_client(num1, num2)
    rospy.loginfo("Result of adding {} and {}: {}".format(num1, num2, result))