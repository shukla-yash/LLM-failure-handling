#!/usr/bin/env python3

import numpy as np
import cv2
# import open3d as o3d
# import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
def find_highest_points(point_cloud, threshold):
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

def find_center_of_points(points):
    """
    Find the center of a set of points.
    
    Parameters:
        points (numpy.ndarray): Array of points
        
    Returns:
        tuple: Coordinates of the center (x, y, z)
    """
    center = np.mean(points, axis=0)
    return center



def create_point_cloud(depth_image, intrinsics):
    """
    Create point cloud from depth image using camera intrinsics.
    
    Parameters:
        depth_image (numpy.ndarray): Depth image
        intrinsics (rs.intrinsics): Camera intrinsics
        
    Returns:
        o3d.geometry.PointCloud: Point cloud
    """
    points = rs.pointcloud()
    points.map_to(depth_image)
    depth_frame = rs.depth_frame(depth_image)
    pc = points.calculate(depth_frame)
    
    # Extract vertices and reshape to (N, 3)
    vertices = np.asanyarray(pc.get_vertices())
    vertices = vertices.view(np.float32).reshape(-1, 3)
    
    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    return pcd

def visualize_point_cloud(point_cloud):
    """
    Visualize point cloud using Open3D.
    
    Parameters:
        point_cloud (o3d.geometry.PointCloud): Point cloud
    """
    o3d.visualization.draw_geometries([point_cloud])

def apply_bounding_box(point_cloud, point1, point2, intrinsics, depth_image):
    """
    Apply bounding box to point cloud.
    
    Parameters:
        point_cloud (o3d.geometry.PointCloud): Point cloud
        bounding_box (list): List of corner points of bounding box
        
    Returns:
        o3d.geometry.PointCloud: Bounded point cloud
    """
    x1, y1 = point1
    x2, y2 = point2

    x1_3d = (x1 - intrinsics.ppx) * depth_image[y1,x1] / (intrinsics.fx*1000)
    y1_3d = (y1 - intrinsics.ppy) * depth_image[y1,x1] / (intrinsics.fy*1000)
    x2_3d = (x2 - intrinsics.ppx) * depth_image[y2,x2] / (intrinsics.fx*1000)
    y2_3d = (y2 - intrinsics.ppy) * depth_image[y2,x2] / (intrinsics.fy*1000)

    # min_bound = np.min(bounding_box, axis=0)
    # max_bound = np.max(bounding_box, axis=0)
    points = np.asarray(point_cloud.points)
    indices = np.where((points[:, 0] >= x1_3d) & (points[:, 0] <= x2_3d) &
                       (points[:, 1] >= y1_3d) & (points[:, 1] <= y2_3d))
    bounded_point_cloud = o3d.geometry.PointCloud()
    bounded_point_cloud.points = o3d.utility.Vector3dVector(points[indices])
    return bounded_point_cloud

def publish_point():
    # Initialize the ROS node
    rospy.init_node('point_publisher', anonymous=True)

    # Create a publisher for the Marker message on the topic "/visualization_marker"
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "camera_color_optical_frame"  # Set the frame of the marker
    marker.type = Marker.SPHERE  # Set the marker type to a sphere
    marker.action = Marker.ADD  # Set the marker action to add
    marker.scale.x = 0.02  # Set the scale of the marker
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.a = 1.0  # Set the alpha (transparency) of the marker
    marker.color.r = 1.0  # Set the color of the marker (red)
    marker.pose.orientation.w = 1.0  # Set the orientation of the marker

    # Rate at which to publish the message (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a Point message representing the position of the marker
        point = Point()
        point.x = 0.01250409  # Set the x-coordinate
        point.y = -0.14084522  # Set the y-coordinate
        point.z = 0.53286652  # Set the z-coordinate

        # Set the position of the marker
        marker.pose.position = point

        # Set the timestamp of the marker
        marker.header.stamp = rospy.Time.now()

        # Publish the Marker message
        pub.publish(marker)

        # Log the published message
        rospy.loginfo("Publishing Marker at Point: x={}, y={}, z={}".format(point.x, point.y, point.z))

        # Sleep to maintain the specified rate
        rate.sleep()

# # Initialize RealSense pipeline
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# # Start streaming
# pipeline.start(config)
# frames = pipeline.wait_for_frames()
# align = rs.align(rs.stream.color)

# aligned_frames = align.process(frames)
# depth_frame = aligned_frames.get_depth_frame()
# color_frame = aligned_frames.get_color_frame()

# # depth_frame = frames.get_depth_frame()
# # color_frame = frames.get_color_frame()

# # Get camera intrinsics
# intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

# # aligned_depth_frame, aligned_color_frame = align_frames(depth_frame, color_frame, intrinsics)
# # depth_frame, color_frame = align_frames(depth_frame, color_frame, intrinsics)

# color_image = np.asanyarray(color_frame.get_data())
# cv2.imwrite("rgb_image.jpg", color_image)

# # Save depth image
# depth_image = np.asanyarray(depth_frame.get_data())
# cv2.imwrite("depth_image.png", depth_image)

# # Create point cloud
# point_cloud = create_point_cloud(depth_frame, intrinsics)

# visualize_point_cloud(point_cloud)


# point1 = (280,100)
# point2 = (380,210)

# # Apply bounding box
# bounded_point_cloud = apply_bounding_box(point_cloud, point1, point2, intrinsics, depth_image)

# # Visualize point cloud
# visualize_point_cloud(bounded_point_cloud)


# # Example point cloud (replace with your data)
# point_cloud = np.asarray(bounded_point_cloud.points)  # Random point cloud
# threshold = 0.02  # Threshold for considering z-values

# # Find points with highest z-value
# highest_points = find_highest_points(point_cloud, threshold)

# # Find center of highest points
# center = find_center_of_points(highest_points)

# print("Center of highest points:", center)

# single_point_cloud = o3d.geometry.PointCloud()
# single_point_cloud.points = o3d.utility.Vector3dVector([center])

# # Define a color for the single point (here we use red)
# color = [0, 1, 0]

# # Assign the color to the single point cloud
# single_point_cloud.colors = o3d.utility.Vector3dVector([color])
# o3d.visualization.draw_geometries([bounded_point_cloud, single_point_cloud])

# # Stop streaming
# pipeline.stop()


if __name__ == '__main__':
    try:
        publish_point()
    except rospy.ROSInterruptException:
        pass