import numpy as np
import math
import open3d as o3d

class LidarOdometry:

    def __init__(self):
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

    def odometry(self, point_cloud):
        
        self.counter += 1
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the 3D data
        points = data[:, :-1]
        points = self.filter_points_fast(points)
        #points[:, 2] = -points[:, 2]

        # w dont use first 10 scans because they may have alot of noise
        if 1 <= self.counter:
            # get lidar transform of current step
            # lidar_tran = self.lidar_transform_list[self.counter]
            lidar_tran = point_cloud.transform
            # find ground truth position of current step and save it in a list
            pose_gt = [lidar_tran.location.x, lidar_tran.location.y, lidar_tran.location.z]

        if self.counter == 1:
            self.T1 = lidar_tran.get_matrix()
            return pose_gt

        elif self.counter == 2:
            self.T2 = lidar_tran.get_matrix()
            
            # In this step we should save pointcloud as a source pointcloud 
            # for registration algorithm that will be used in next step
            self.source = o3d.geometry.PointCloud()
            self.source.points = o3d.utility.Vector3dVector(points)
            if self.voxel_size != 0:
                self.source = self.source.voxel_down_sample(voxel_size= self.voxel_size)

            # We know that T2 = T1 * T_rel_21 so we can compute T_rel_21:
            T_rel_21 = np.linalg.inv(self.T1) @ self.T2

            # also from constant velocity assumption we have:
            #   -->  T_rel_32 = T_rel_21
            self.T_rel_32 = T_rel_21

            return pose_gt

        elif 2<self.counter:

            T_rel = self.registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            # T3 that transform scan3 to global coordinate is obtained by:
            #   --->  T3 = T2 * T_rel_32
            T3 = self.T2 @ self.T_rel_32

            self.T2 = T3
            return np.ndarray.tolist(T3[:3,-1])
    
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
    
    '''def sampling(self, pcd):
        # Estimate the ground plane using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

        # Extract ground points
        ground_points = pcd.select_by_index(inliers)

        # Remove ground points from the point cloud
        non_ground_points = pcd.select_by_index(inliers, invert=True)

        non_ground_points = non_ground_points.voxel_down_sample(voxel_size= self.voxel_size)

        return non_ground_points'''