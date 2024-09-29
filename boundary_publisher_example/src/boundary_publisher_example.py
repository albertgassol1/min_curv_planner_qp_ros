#!/usr/bin/env python

import rospy
import numpy as np
from min_curv_msgs.msg import Paths  # Importing your custom message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from typing import List
import rospkg
from pathlib import Path
from scipy.interpolate import interp1d
from scipy.spatial import KDTree

class BoundaryPublisher:
    def __init__(self):
        # Load parameters from the parameter server
        self.publish_frequency = 1  # Default value
        boundaries_topic = rospy.get_param("topics/boundaries")

        # Create a publisher for the custom Path message
        self.path_pub = rospy.Publisher(boundaries_topic, Paths, queue_size=10)
        self.left_boundary_pub = rospy.Publisher("visualization/example/left_boundary", PathMsg, queue_size=10)
        self.right_boundary_pub = rospy.Publisher("visualization/example/right_boundary", PathMsg, queue_size=10)
        self.centerline_pub = rospy.Publisher("visualization/example/centerline", PathMsg, queue_size=10)

        self.index = 0

        # Set the frame_id for publishing
        self.frame_id = rospy.get_param("frames/world")
        
        rospack = rospkg.RosPack()
        package_path = Path(rospack.get_path('boundary_publisher_example'))
        right_boundary = np.loadtxt(package_path / 'data/right_boundary.txt')
        left_boundary = np.loadtxt(package_path / 'data/left_boundary.txt')
        
        right_boundary_distance = np.cumsum(np.sqrt(np.sum(np.diff(right_boundary, axis=0)**2, axis=1 )))
        right_boundary_distance = np.insert(right_boundary_distance, 0, 0) / right_boundary_distance[-1]
        left_boundary_distance = np.cumsum(np.sqrt(np.sum(np.diff(left_boundary, axis=0)**2, axis=1 )))
        left_boundary_distance = np.insert(left_boundary_distance, 0, 0) / left_boundary_distance[-1]
        self.u = np.linspace(0, 1, 200)
        
        interpolator_right = interp1d(right_boundary_distance, right_boundary, kind='cubic', axis=0)
        interpolator_left = interp1d(left_boundary_distance, left_boundary, kind='cubic', axis=0)
        self.right_boundary = interpolator_right(self.u)
        self.left_boundary = interpolator_left(self.u)
        
        # Create a KDTree for the right boundary
        self.right_boundary_tree = KDTree(self.right_boundary)
        
        self.num_points = rospy.get_param("optimizer/num_control_points")
        
    def get_poses(self, points: np.ndarray) -> List[PoseStamped]:
        poses = []
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            poses.append(pose)
        return poses
    @staticmethod
    def find_nearest_point(tree, point):
        _, idx = tree.query(point)
        return idx
    
    @staticmethod
    def slice(starting_index, ending_index, length):
        if starting_index < ending_index:
            return np.arange(starting_index, ending_index)
        else:
            return np.concatenate([np.arange(starting_index, length), np.arange(0, ending_index)])
        

    def publish_paths(self):
        rate = rospy.Rate(self.publish_frequency)  # Set the publishing frequency
        while not rospy.is_shutdown():
            # Create an instance of the custom Path message
            path_msg = Paths()
            path_msg.header.frame_id = self.frame_id
            path_msg.header.stamp = rospy.Time.now()
            
            # Get points from the right boundary
            right_boundary = self.right_boundary[self.slice(self.index, (self.index + self.num_points) % len(self.right_boundary), len(self.right_boundary))]
            # Get points from the left boundary
            left_index = self.find_nearest_point(self.right_boundary_tree, right_boundary[0])
            left_boundary = self.left_boundary[self.slice(left_index, (left_index + self.num_points) % len(self.left_boundary), len(self.left_boundary))]
            self.index = (self.index + 1) % len(self.left_boundary)
            # Get points from the centerline, middle of the right and left boundaries
            centerline = (right_boundary + left_boundary) / 2
                        
            # Set the left boundary
            path_msg.left_boundary.header.frame_id = self.frame_id
            path_msg.left_boundary.header.stamp = rospy.Time.now()
            path_msg.left_boundary.poses = self.get_poses(left_boundary)

            # Set the right boundary
            path_msg.right_boundary.header.frame_id = self.frame_id
            path_msg.right_boundary.header.stamp = rospy.Time.now()
            path_msg.right_boundary.poses = self.get_poses(right_boundary)

            # Set the centerline
            path_msg.centerline.header.frame_id = self.frame_id
            path_msg.centerline.header.stamp = rospy.Time.now()
            path_msg.centerline.poses = self.get_poses(centerline)

            # Publish the custom path message
            self.path_pub.publish(path_msg)
            
            # Publish visualization paths
            self.left_boundary_pub.publish(path_msg.left_boundary)
            self.right_boundary_pub.publish(path_msg.right_boundary)
            self.centerline_pub.publish(path_msg.centerline)

            rospy.loginfo(f"[boundary_publisher_node] Published path {self.index - 1}")
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('boundary_publisher_node', anonymous=True)
    boundary_publisher = BoundaryPublisher()
    boundary_publisher.publish_paths()
