import rospy
from realsense_segment2.srv import FindGraspingPoint, FindGraspingPointResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

from PIL import Image as pil_img
import ros_numpy
from sensor_msgs.msg import Image
import cv2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import sys
sys.path.append("/home/unicorn/catkin_ws/src/realsense_segment2/src")
from object_mask import LangSAM
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
# from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class ImageProcessor:
    def __init__(self):
        self.color_image_np = None
        self.depth_image_np = None
        self.point_cloud_data = None
        self.gdino_sam_model = LangSAM()

        rospy.loginfo("getting here 34")
        # self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


        # Subscribe to the color image topic
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)

        # Subscribe to the depth image topic
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        # Subscribe to the depth image topic
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.point_cloud_callback)

        rospy.Service('find_grasping_point', FindGraspingPoint, self.find_grasping_point)
        
        # rospy.loginfo("color np shape: ", self.color_image_np.shape)
        # rospy.loginfo("depth np shape:", self.depth_image_np.shape)

        rospy.loginfo("here123")

        # self.process_images()

    def color_image_callback(self, msg):
        # rospy.loginfo("color image check")
        try:
            self.color_image_np = np.asarray(ros_numpy.numpify(msg), dtype= np.float64)
            # self.color_image_np = self.color_image_np[..., ::-1]
            # cv2.imwrite('/home/unicorn/catkin_ws/src/realsense_segment2/src/color_img.jpg', self.color_image_np)
        except Exception as e:
            print(e)

    def depth_image_callback(self, msg):
        # rospy.loginfo("depth image check")
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image_np = np.asarray(ros_numpy.numpify(msg))
            # self.process_images()

        except Exception as e:
            print(e)

    def point_cloud_callback(self, msg):

        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            points.append([point[0], point[1], point[2]])  # Extract (x, y, z) coordinates

        # Create Open3D point cloud object
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Visualize the point cloud
        # o3d.visualization.draw_geometries([pcd])        
        self.point_cloud_data = np.asarray(pcd.points)


    def process_images(self, object_string):
        rospy.loginfo("got string, %s", object_string)
        # rospy.loginfo("color np shape: ", self.color_image_np)
        # rospy.loginfo("depth np shape:", self.depth_image_np)

        while self.color_image_np is None and self.depth_image_np is None and self.point_cloud_data is None:
            rospy.sleep()

        if self.color_image_np is not None and self.depth_image_np is not None:
            rospy.loginfo("here3")
            print("color image shape: ", self.color_image_np.shape)
            print("depth image shape: ", self.depth_image_np.shape)
            bbox, masked_image = self.gdino_sam_model.get_mask(pil_img.fromarray(np.uint8(self.color_image_np)), pil_img.fromarray(self.depth_image_np), object_string)
            # bbox, masked_image = self.gdino_sam_model.get_mask("/home/unicorn/catkin_ws/src/realsense_segment2/src/color_img.jpg", pil_img.fromarray(self.depth_image_np), object_string)

            rospy.loginfo("here4")
        if self.point_cloud_data is not None:
            rospy.loginfo("here2")
            x1, y1, x2, y2 = bbox

            x1_3d = (x1 - 652.453) * self.depth_image_np[y1,x1] / (645.366*1000)
            y1_3d = (y1 - 363.437) * self.depth_image_np[y1,x1] / (644.544*1000)
            x2_3d = (x2 - 652.453) * self.depth_image_np[y2,x2] / (645.366*1000)
            y2_3d = (y2 - 363.437) * self.depth_image_np[y2,x2] / (644.544*1000)

            points = self.point_cloud_data
            indices = np.where((points[:, 0] >= x1_3d) & (points[:, 0] <= x2_3d) &
                            (points[:, 1] >= y1_3d) & (points[:, 1] <= y2_3d))

            points = points[indices]

            bounded_point_cloud = o3d.geometry.PointCloud()
            point_cloud = np.reshape(points, (-1, 3))

            bounded_point_cloud.points = o3d.utility.Vector3dVector(point_cloud)

            # Example point cloud (replace with your data)
            point_cloud = np.asarray(bounded_point_cloud.points)  # Random point cloud
            threshold = 0.02  # Threshold for considering z-values

            zero_indices = np.where((point_cloud[:, 0] == 0) & (point_cloud[:, 1] == 0) & (point_cloud[:, 2] == 0))[0]
            point_cloud = np.delete(point_cloud, zero_indices, axis=0)


            # Find points with highest z-value
            highest_points = self.find_highest_points(point_cloud, threshold)

            # Find center of highest points
            center = self.find_center_of_points(highest_points)
            # print(center)

            single_point_cloud = o3d.geometry.PointCloud()
            single_point_cloud.points = o3d.utility.Vector3dVector([center])

            # Define a color for the single point (here we use red)
            color = [0, 1, 0]

            # Assign the color to the single point cloud
            single_point_cloud.colors = o3d.utility.Vector3dVector([color])
            # o3d.visualization.draw_geometries([bounded_point_cloud, single_point_cloud])

            pose_stamped = PoseStamped()
            # pose_stamped = Pose()

            point = Point()
            orientation = Quaternion()
# 0.539, 0.087, 0.634
            point.x = center[0]  # Set the x-coordinate
            point.y = center[1] # Set the y-coordinate
            point.z = center[2]  # Set the z-coordinate
            pose_stamped.pose.position = point
            orientation.x = 0
            orientation.y = 0
            orientation.z = 0
            orientation.w = 1       

            pose_stamped.pose.orientation = orientation

            pose_stamped.header.frame_id = "camera_color_optical_frame"  # Set the frame of the marker
            # pose_stamped.header.frame_id = "base_link"  # Set the frame of the marker

            pose_stamped.header.stamp = rospy.Time.now()


            # marker = Marker()
            # marker.header.frame_id = "camera_color_optical_frame"  # Set the frame of the marker
            # marker.type = Marker.SPHERE  # Set the marker type to a sphere
            # marker.action = Marker.ADD  # Set the marker action to add
            # marker.scale.x = 0.02  # Set the scale of the marker
            # marker.scale.y = 0.02
            # marker.scale.z = 0.02
            # marker.color.a = 1.0  # Set the alpha (transparency) of the marker
            # marker.color.r = 1.0  # Set the color of the marker (red)
            # marker.pose.orientation.w = 1.0  # Set the orientation of the marker

            # point = Point()

            # # Set the position of the marker
            # marker.pose.position = point

            # Set the timestamp of the marker
            # marker.header.stamp = rospy.Time.now()

            # # Publish the Marker message
            # self.pub.publish(marker)

            # # Log the published message
            rospy.loginfo("Publishing Marker at Point: x={}, y={}, z={}".format(point.x, point.y, point.z))

            return FindGraspingPointResponse(pose_stamped)



    def find_highest_points(self, point_cloud, threshold):
        """
        Find points with the highest z-value in the point cloud.
        
        Parameters:
            point_cloud (numpy.ndarray): Point cloud of shape (n, 3)
            threshold (float): Threshold for considering z-values
        
        Returns:
            numpy.ndarray: Array of points with highest z-value satisfying the threshold
        """
        max_z = np.min(point_cloud[:, 2])
        # mask = point_cloud[:, 2] >= max_z - 0.01
        mask = point_cloud[:, 2] <= max_z + 0.04
        highest_points = point_cloud[mask]
        return highest_points

    def find_center_of_points(self, points):
        """
        Find the center of a set of points.
        
        Parameters:
            points (numpy.ndarray): Array of points
            
        Returns:
            tuple: Coordinates of the center (x, y, z)
        """
        center = np.mean(points, axis=0)
        return center

    def find_grasping_point(self, req):
        object_string = req.object_string
        rospy.loginfo("here: %s",object_string)
        while self.color_image_np is None and self.depth_image_np is None and self.point_cloud_data is None:
            rospy.sleep()        
        return self.process_images(object_string)

        # marker = image_processor.process_images(object_string)
        # # rospy.sleep(10000)
        # return marker
        # if marker is not None:
        #     print("marker at Point: x={}, y={}, z={}".format(marker.pose.position.point.x, marker.pose.position.point.y, marker.pose.position.point.z))
        #     return marker    

# def find_grasping_point(req):
#     object_string = req.object_string
#     rospy.loginfo("here: %s ",object_string)
#     return Marker()

def find_grasping_point_server():
    rospy.init_node('find_grasping_point_server')
    image_processor = ImageProcessor()
    # rospy.Service('find_grasping_point', FindGraspingPoint, find_grasping_point)
    # rospy.loginfo("Ready to find grasping point.")
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('find_grasping_point_server')
    image_processor = ImageProcessor()
    rospy.spin()