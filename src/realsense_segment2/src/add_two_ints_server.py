import rospy
from realsense_segment2.srv import AddTwoInts

def add_two_numbers(req):
    result = req.a + req.b
    rospy.loginfo("Adding {} and {}: Result = {}".format(req.a, req.b, result))
    return result

def add_two_numbers_server():
    rospy.init_node('add_two_numbers_server')
    rospy.Service('add_two_numbers', AddTwoInts, add_two_numbers)
    rospy.loginfo("Ready to add two numbers.")
    rospy.spin()

if __name__ == "__main__":
    add_two_numbers_server()