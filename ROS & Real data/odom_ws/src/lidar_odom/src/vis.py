#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from lidar_odom.msg import location
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

class DynamicTFBroadcaster:

    def __init__(self):
        rospy.init_node('dynamic_tf2_broadcaster')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()

        self.t.header.frame_id = "world"
        self.t.child_frame_id = "base_link"
        self.localization_sub = rospy.Subscriber('localization', location, self.localization_callback)

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        self.rate = rospy.Rate(10.0)
        self.tf_handler()

    def localization_callback(self, msg):
        self.x = msg.x
        self.y = msg.y

        self.roll = msg.roll
        self.yaw = msg.yaw


    def tf_handler(self):

        while not rospy.is_shutdown():
            x, y, z = self.x, self.y, self.z
            roll, pitch, yaw = self.roll, self.pitch, self.yaw
            current_time = rospy.Time.now()

            # odometry
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "world"
            odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))
            odom.child_frame_id = "base_link"
            # tf
            self.t.header.stamp = current_time
            self.t.transform.translation.x = x
            self.t.transform.translation.y = y
            self.t.transform.translation.z = 0.0
            self.t.transform.rotation.x = odom_quat[0]
            self.t.transform.rotation.y = odom_quat[1]
            self.t.transform.rotation.z = odom_quat[2]
            self.t.transform.rotation.w = odom_quat[3]


            self.odom_pub.publish(odom)
            self.br.sendTransform(self.t)
            self.rate.sleep()

            
if __name__ == '__main__':

    tf_han = DynamicTFBroadcaster()
'''
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

class CameraListner():
    def __init__(self):
        self.pc = None
        self.n = 0

        # odometry variables
        self.counter = 0
        self.point_cl = None
        self.voxel_size = 0.5
     
        self.threshold = 1.3
        #self.threshold = 1.3
        rospack = rospkg.RosPack()
        csv_path = os.path.join(rospack.get_path("lidar_odom"), "gt_data", "gt.csv")
        self.gt = pd.read_csv(csv_path).loc[:1672]

        rospy.init_node('vis', anonymous=True)
        rospy.Subscriber('/ouster/points', PointCloud2, self.callback)
        rospy.Subscriber('/odom', PointCloud2, self.callback)
        #self.pub = rospy.Publisher("/pcl2", PointCloud2, queue_size=10)
        self.vis_show()
    
    def callback(self, points):
        self.pc = points
        self.n = self.n + 1
        #self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc))
        points_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc)

        points = self.filter_points_fast(points_array)
        self.point_cl = o3d.geometry.PointCloud()
        self.point_cl.points = o3d.utility.Vector3dVector(points)
        if self.voxel_size != 0:
            self.point_cl = self.point_cl.voxel_down_sample(voxel_size= self.voxel_size)
            #print(type(np.array(self.point_cl.points)))
        #pcl2 = ros_numpy.point_cloud2.array_to_pointcloud2(points, frame_id='OS1')
        #self.pub.publish(pcl2)


    def vis_show(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Carla Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True
        frame = 0

        while True:
            if self.point_cl == None:
                continue
            #if frame == 2:
            vis.add_geometry(self.point_cl)
            vis.update_geometry(self.point_cl)
            rospy.loginfo('KKKKKKK')
            vis.poll_events()
            vis.update_renderer()
            # # This can fix Open3D jittering issues:
            #time.sleep(0.02)
            frame += 1


    def filter_points_fast(self, points_array):
        # non-ground selection
        z_condition = np.logical_and(points_array[:, 2] > -1.2, points_array[:, 2] < 1.2)
        y_condition = np.logical_and(points_array[:, 1] > -60, points_array[:, 1] < 60)
        x_condition = np.logical_and(points_array[:, 0] > -60, points_array[:, 0] < 60)
        filtered_indices = np.logical_and(z_condition, x_condition)
        filtered_indices = np.logical_and(filtered_indices, y_condition)
        
        #y_condition2 = np.logical_or(points_array[:, 1] <= -1.2, points_array[:, 1] >= 1.2)
        #x_condition2 = np.logical_or(points_array[:, 0] <= -2.8, points_array[:, 0] >= 2)
        #exclude_condition = np.logical_or(y_condition2, x_condition2)
        
        #filtered_indices = np.logical_and(exclude_condition, filtered_indices)
        
        filtered_points = points_array[filtered_indices]
        return filtered_points

if __name__ == '__main__':

    listener = CameraListner()
    #updater = ViewerWidget(listener)
    rospy.spin()'''