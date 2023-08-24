#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
# from std_msgs.msg import Header
from lidar_odom.msg import location
import os, rospkg
import time

import tf
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from scipy.spatial.transform import Rotation
import tf2_ros
import tf2_msgs.msg

class CameraListner():
    def __init__(self):
        self.pc = None
        self.current_time = 0
        self.n = 0

        # odometry variables
        self.counter = 0
        self.pcls_list = []
        self.lidar_transform_list = []
        self.T1 = None
        self.T2 = None
        self.T_rel_32 = None
        self.pos_gt = []
        self.pos_est = []

        # registration variable
        self.source = None
        # with down sampling
        self.voxel_size = 0.5
        self.threshold = 1.3
        #self.threshold = 1.3
        '''rospack = rospkg.RosPack()
        csv_path = os.path.join(rospack.get_path("lidar_odom"), "gt_data", "gt.csv")
        self.gt = pd.read_csv(csv_path).loc[:1672]'''

        rospy.init_node('laser_to_pcd', anonymous=True)
        rospy.Subscriber('/ouster/points', PointCloud2, self.callback)
        self.pub = rospy.Publisher("localization", location, queue_size=10)
        self.pub_cloud = rospy.Publisher("mycloud", PointCloud2, queue_size=10)
        #self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    def callback(self, points):
        self.pc = points
        self.n = self.n + 1
        #self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc))
        points_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc)
        self.odometry(points_array, self.pc)
        
    def tf_creator(self):
        self.current_time = rospy.Time.now()
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = self.current_time
        t.child_frame_id = "sensor"
        t.transform.translation.x = 0.1
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def odometry(self, point_cloud, my_pc):
        self.counter += 1
        self.tf_creator()

        points = self.filter_points_fast(point_cloud)
        
        '''my_R = np.array([[0.5, 0.86602, 0],
                         [-0.86602, 0.5, 0],
                         [0, 0, 1]])
        points = points@my_R'''
        #rospy.loginfo(len(points))
        # create msg
        loc = location()
        # Create a Header object
        '''header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        loc.header = header'''

        if self.counter == 1:
            # create Rot matrix
            self.T1 = np.eye(4)
            loc.x, loc.y, loc.z, loc.roll, loc.pitch, loc.yaw = [0, 0, 0, 0, 0, 0]
            self.pub.publish(loc)

        elif self.counter == 2:   
            # create Rot matrix
            self.T2 = np.eye(4)
            # In this step we should save pointcloud as a source pointcloud 
            # for registration algorithm that will be used in next step
            self.source = o3d.geometry.PointCloud()
            self.source.points = o3d.utility.Vector3dVector(points)
            if self.voxel_size != 0:
                self.source = self.source.voxel_down_sample(voxel_size= self.voxel_size)
            
            T_rel_21 = np.linalg.inv(self.T1) @ self.T2
            self.T_rel_32 = T_rel_21

            loc.x, loc.y, loc.z, loc.roll, loc.pitch, loc.yaw = [0, 0, 0, 0, 0, 0]
            #loc.x_gt, loc.y_gt, loc.z_gt = pos_gt
            self.pub.publish(loc)

        elif 2<self.counter:
            T_rel = self.registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            # T3 that transform scan3 to global coordinate is obtained by:
            #   --->  T3 = T2 * T_rel_32
            T3 = self.T2 @ self.T_rel_32

            self.T2 = T3
            r = Rotation.from_matrix(T3[:3,:3])
            euler_angles = r.as_euler('xyz', degrees=False)  # 'xyz' corresponds to roll-pitch-yaw
            roll, pitch, yaw = euler_angles
            loc.x, loc.y, loc.z = np.ndarray.tolist(T3[:3,-1])
            loc.roll, loc.pitch, loc.yaw = [roll, pitch, yaw]
            #loc.x_gt, loc.y_gt, loc.z_gt = pos_gt
            self.pub.publish(loc)
        
        my_pc.header.frame_id = "sensor"
        my_pc.header.stamp = self.current_time
        self.pub_cloud.publish(my_pc)

    def filter_points_fast(self, points_array):
        # non-ground selection
        z_condition = np.logical_and(points_array[:, 2] > 0.2, points_array[:, 2] < 1)
        y_condition = np.logical_and(points_array[:, 1] > -3, points_array[:, 1] < 3)
        x_condition = np.logical_and(points_array[:, 0] > -30, points_array[:, 0] < 30)
        filtered_indices = np.logical_and(z_condition, x_condition)
        filtered_indices = np.logical_and(filtered_indices, y_condition)
        
        '''y_condition2 = np.logical_or(points_array[:, 1] <= -1.2, points_array[:, 1] >= 1.2)
        x_condition2 = np.logical_or(points_array[:, 0] <= -2.8, points_array[:, 0] >= 2)
        exclude_condition = np.logical_or(y_condition2, x_condition2)
        
        filtered_indices = np.logical_and(exclude_condition, filtered_indices)'''
        
        filtered_points = points_array[filtered_indices]
        return filtered_points

    def registration(self, data):
        # first we should prepare traget pcl
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(data)
        #print("####len222###:", len(data[:, :-1]))
        target_norm = None
        if self.voxel_size == 0:
            # without down sampling
            target_norm = target
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=10))
        else:
            # with down sampling
            target = target.voxel_down_sample(voxel_size= self.voxel_size)
            target_norm = target
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.15, max_nn=30))
            #target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.4, max_nn=10))

        # we should give an initial transform matrix to registration algorithm
        initial_transform = np.linalg.inv(self.T_rel_32)
        #initial_transform = self.T_rel_32
        
        # point to plane ICP registraion algorithm
        reg_p2l = o3d.pipelines.registration.registration_icp(
        self.source, target_norm, self.threshold, init =initial_transform,
        estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                                                        relative_rmse=0.01,
                                                                        max_iteration=300))
        # The source we used here is a target for next iteration
        self.source = target

        return reg_p2l.transformation

    #def my_tf_loop(self):


if __name__ == '__main__':

    listener = CameraListner()
    #updater = ViewerWidget(listener)
    rospy.spin()