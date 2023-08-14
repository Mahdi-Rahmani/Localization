#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros_numpy
import numpy as np
# from std_msgs.msg import Header
from lidar_odom.msg import location

class CameraListner():
    def __init__(self):
        self.pc = None
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

        rospy.init_node('laser_to_pcd', anonymous=True)
        rospy.Subscriber('/os1_cloud_node/points', PointCloud2, self.callback)
        self.pub = rospy.Publisher("localization", location, queue_size=10)

    
    def callback(self, points):
        self.pc = points
        self.n = self.n + 1
        #self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc))
        points_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pc)
        self.odometry(points_array)
        
    
    def odometry(self, point_cloud):
        
        self.counter += 1

        points = self.filter_points_fast(point_cloud)
        #rospy.loginfo(len(points))
        # create msg
        loc = location()
        # Create a Header object
        '''header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        loc.header = header'''

        if self.counter == 1:
            
            # In this step we should save pointcloud as a source pointcloud 
            # for registration algorithm that will be used in next step
            self.source = o3d.geometry.PointCloud()
            self.source.points = o3d.utility.Vector3dVector(points)
            if self.voxel_size != 0:
                self.source = self.source.voxel_down_sample(voxel_size= self.voxel_size)
            self.T2 = np.eye(4)
            self.T_rel_32 = np.eye(4)

            loc.x, loc.y, loc.z = [0, 0, 0]
            self.pub.publish(loc)

        elif 1<self.counter:

            T_rel = self.registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            # T3 that transform scan3 to global coordinate is obtained by:
            #   --->  T3 = T2 * T_rel_32
            T3 = self.T2 @ self.T_rel_32

            self.T2 = T3

            
            loc.x, loc.y, loc.z = np.ndarray.tolist(T3[:3,-1])
            self.pub.publish(loc)


    def filter_points_fast(self, points_array):
        # non-ground selection
        z_condition = np.logical_and(points_array[:, 2] > -0.2, points_array[:, 2] < 0.3)
        x_condition = np.logical_and(points_array[:, 0] > -25, points_array[:, 0] < 20)
        filtered_indices = np.logical_and(z_condition, x_condition)
        
        y_condition2 = np.logical_or(points_array[:, 1] <= -1.2, points_array[:, 1] >= 1.2)
        x_condition2 = np.logical_or(points_array[:, 0] <= -2.8, points_array[:, 0] >= 2)
        exclude_condition = np.logical_or(y_condition2, x_condition2)
        
        filtered_indices = np.logical_and(exclude_condition, filtered_indices)
        
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
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.15, max_nn=10))


        # we should give an initial transform matrix to registration algorithm
        initial_transform = np.linalg.inv(self.T_rel_32)
        
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









'''

class ViewerWidget(QtWidgets.QWidget):
    def __init__(self, subscriber, parent=None):
        self.subscriber = subscriber
        rospy.loginfo('initialization')

        self.vis = open3d.visualization.Visualizer()
        self.point_cloud = None
        self.updater()

    ############################################################################
    def updater(self):
        rospy.loginfo('start')
        self.first = False
        while (self.subscriber.pc is None):
            time.sleep(2)
        self.point_cloud = open3d.geometry.PointCloud()
        self.point_cloud.points = open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
        self.vis.create_window()
        print('get points')
        #self.vis.add_geometry(self.point_cloud)
        print ('add points')
        self.vis.update_geometry(self.point_cloud)
        self.vis.poll_events()

        self.vis.update_renderer()

        while not rospy.is_shutdown():
            self.point_cloud.points =  open3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.subscriber.pc))
            self.vis.update_geometry(self.point_cloud)
            self.vis.poll_events()
            self.vis.update_renderer()'''

if __name__ == '__main__':

    listener = CameraListner()
    #updater = ViewerWidget(listener)
    rospy.spin()