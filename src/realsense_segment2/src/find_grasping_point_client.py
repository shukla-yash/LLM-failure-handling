#!/home/unicorn/miniconda3/envs/kinovaconda/bin/python

# import rospy
# from realsense_segment2.srv import FindGraspingPoint
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import PoseStamped


# def find_grasping_point_client(object_string):
#     rospy.wait_for_service('find_grasping_point')
#     find_grasping_point = rospy.ServiceProxy('find_grasping_point', FindGraspingPoint)
#     response = find_grasping_point(object_string)
#     return response
#     # try:
#     # except rospy.ServiceException as e:
#     #     rospy.logerr("Service call failed: %s", e)

# if __name__ == "__main__":
#     rospy.init_node('find_grasping_point_client')
#     pub = rospy.Publisher('/viz_pose', PoseStamped, queue_size=10)
#     object_string = input("Enter object of interest: ")
#     result = find_grasping_point_client(object_string)
#     rospy.loginfo("Publishing {}".format(result.pose_stamped))
#     pub.publish(result.pose_stamped)
#     rospy.spin()        


import rospy
from realsense_segment2.srv import FindGraspingPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

def find_grasping_point_client(object_string):
    rospy.wait_for_service('find_grasping_point')
    find_grasping_point = rospy.ServiceProxy('find_grasping_point', FindGraspingPoint)
    response = find_grasping_point(object_string)
    return response

def transform_pose(pose_stamped, target_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        # Wait for the transformation to become available
        tf_buffer.can_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(), rospy.Duration(1.0))
        transform = tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(), rospy.Duration(1.0))

        # Transform the pose
        transformed_pose = do_transform_pose(pose_stamped, transform)
        return transformed_pose
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to transform pose to {}".format(target_frame))
        return None


def main():
    rospy.init_node('find_grasping_point_client')
    object_string = input("Enter object of interest: ")
    result = find_grasping_point_client(object_string)
    transformed_pose = transform_pose(result.pose_stamped, "base_link")
    transformed_pose.pose.orientation.x = 1
    transformed_pose.pose.orientation.y = 0
    transformed_pose.pose.orientation.z = 0
    transformed_pose.pose.orientation.w = 0

    upper_point = PoseStamped()
    upper_point.pose.position.x = transformed_pose.pose.position.x
    upper_point.pose.position.y = transformed_pose.pose.position.y
    upper_point.pose.position.z = transformed_pose.pose.position.z + 0.1
    upper_point.pose.orientation.x = 1
    upper_point.pose.orientation.y = 0
    upper_point.pose.orientation.z = 0
    upper_point.pose.orientation.w = 0
    upper_point.header.frame_id = "base_link"  # Set the frame of the marker
    upper_point.header.stamp = rospy.Time.now()

    # Create a publisher to republish the pose
    pub_lower = rospy.Publisher('/viz_pose_lower', PoseStamped, queue_size=10)
    pub_upper = rospy.Publisher('/viz_pose_upper', PoseStamped, queue_size=10)

    # Function to continuously publish the pose
    def publish_pose():
        rospy.loginfo("Publishing {}".format(transformed_pose))
        pub_lower.publish(transformed_pose)
        rospy.loginfo("Publishing {}".format(upper_point))
        pub_upper.publish(upper_point)
        

    # Continuously publish the pose at a fixed rate
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_pose()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
