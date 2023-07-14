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
        self.voxel_size = 4
        self.threshold = 2

    def odometry(self, point_cloud):
        
        self.counter += 1
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the 3D data
        points = data[:, :-1]
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
    
    def registration(self, data):
        # first we should prepare traget pcl
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(data)
        target_norm = None
        if self.voxel_size == 0:
            # without down sampling
            target_norm = target
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.2, max_nn=130))
        else:
            # with down sampling
            target = target.voxel_down_sample(voxel_size= self.voxel_size)
            target_norm = target
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=4, max_nn=20))


        # we should give an initial transform matrix to registration algorithm
        initial_transform = np.linalg.inv(self.T_rel_32)
        
        # point to plane ICP registraion algorithm
        reg_p2l = o3d.pipelines.registration.registration_icp(
        self.source, target_norm, self.threshold, init =initial_transform,
        estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                                                        relative_rmse=0.0000001,
                                                                        max_iteration=200))
        # The source we used here is a target for next iteration
        self.source = target

        return reg_p2l.transformation
    