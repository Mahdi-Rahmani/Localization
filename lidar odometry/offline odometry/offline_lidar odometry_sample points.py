# importing libraries
import glob
import os
import sys
import time
from datetime import datetime
import random
import numpy as np
import open3d as o3d
import math
import matplotlib.pyplot as plt

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


class OfflineLidarOdometry():
    def __init__(self):
        # carla host and port
        self.host = '127.0.0.1'
        self.port = 3000

        # settings of simulation and lidar
        self.delta = 0.1 
        self.no_rendering = False
        self.no_noise = True
        self.no_autopilot = True
        self.show_axis = False
        self.upper_fov = 15.0
        self.lower_fov = -25.0
        self.channels = 64.0
        self.lidar_range = 100.0
        self.points_per_second = 1000000
        self.don_flag = True

        # bluprints those added to carla
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        self.vehicle = None
        self.lidar = None

        # odometry variables
        self.scan_num = 550
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
        # without down sampling
        self.voxel_size = 0.5
        #self.threshold = 0.85
        self.threshold = 1.3
        self.create_environment()

    def set_traffic_light_to_green(self, tl):
        tl.set_state(carla.TrafficLightState.Green)
        tl.freeze(True)  # Freeze the traffic light state to avoid automatic changes

    def set_all_traffic_lights_to_green(self, world):
        traffic_lights = world.get_actors().filter("*traffic_light*")
        for tl in traffic_lights:
            self.set_traffic_light_to_green(tl)

    def create_environment(self):
        client = carla.Client(self.host, self.port)
        client.set_timeout(2.0)
        world = client.get_world()

        try:
            original_settings = world.get_settings()
            settings = world.get_settings()
            traffic_manager = client.get_trafficmanager(8000)
            traffic_manager.set_synchronous_mode(True)

            settings.fixed_delta_seconds = self.delta
            settings.synchronous_mode = True
            settings.no_rendering_mode = self.no_rendering
            world.apply_settings(settings)

            blueprint_library = world.get_blueprint_library()
            vehicle_bp = blueprint_library.find(self.vehicle_name)
            #vehicle_transform = random.choice(world.get_map().get_spawn_points())
            vehicle_transform = world.get_map().get_spawn_points()[5]
            self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
            self.vehicle.set_autopilot(self.no_autopilot)

            lidar_bp = self.generate_lidar_bp(world, blueprint_library, self.delta)
            x = 0.0
            y = 0.0
            z = 0.0
            user_offset = carla.Location(x, y, z)
            lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

            self.point_list = o3d.geometry.PointCloud()
            self.lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            self.lidar.listen(lambda data: self.lidar_callback(data))

            self.set_all_traffic_lights_to_green(world)
            
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
            dt0 = datetime.now()
            while self.don_flag:
                if frame == 2:
                    vis.add_geometry(self.point_list)
                vis.update_geometry(self.point_list)

                vis.poll_events()
                vis.update_renderer()
                # # This can fix Open3D jittering issues:
                time.sleep(0.009)
                world.tick()

                process_time = datetime.now() - dt0
                sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
                sys.stdout.flush()
                dt0 = datetime.now()
                frame += 1
        finally:
            self.locallization()
            world.apply_settings(original_settings)
            traffic_manager.set_synchronous_mode(False)

            self.vehicle.destroy()
            self.lidar.destroy()

    def generate_lidar_bp(self, world, blueprint_library, delta):
        """Generates a CARLA blueprint based on the script parameters"""
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        if self.no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            lidar_bp.set_attribute('noise_stddev', '0.2')

        lidar_bp.set_attribute('upper_fov', str(self.upper_fov))
        lidar_bp.set_attribute('lower_fov', str(self.lower_fov))
        lidar_bp.set_attribute('channels', str(self.channels))
        lidar_bp.set_attribute('range', str(self.lidar_range))
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / self.delta))
        lidar_bp.set_attribute('points_per_second', str(self.points_per_second))
        return lidar_bp

    def lidar_callback(self, point_cloud):
        """ This function is called after each lidar scan and we save
        pcls and lidar_transform in a list. lidar transform contains groud truth """
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))
        # Isolate the 3D data
        points = data[:, :-1]

        # We're negating the y to correclty visualize a world that matches
        # what we see in Unreal since Open3D uses a right-handed coordinate system
        points[:, :1] = -points[:, :1]
        points = self.filter_points_fast(points)
        self.point_list.points = o3d.utility.Vector3dVector(points)


        self.counter +=1
        if self.counter == self.scan_num:
            self.don_flag = False
        self.pcls_list.append(point_cloud)

    def locallization(self):
        odom_num = 0
        for pcl in self.pcls_list:
            odom_num += 1
            if odom_num == len(self.pcls_list):
                break
            self.odometry(pcl,odom_num)
        self.plot_result(np.array(self.pos_est), np.array(self.pos_gt))

    def filter_non_ground(self, points_array):
        z_condition = np.logical_and(points_array[:, 2] > -0.8, points_array[:, 2] < 0.5)
        filtered_points = points_array[z_condition]
        return filtered_points


    def filter_points_fast(self, points_array):
        # non-ground selection
        z_condition = np.logical_and(points_array[:, 2] > -0.2, points_array[:, 2] < 0.3)
        x_condition = np.logical_and(points_array[:, 0] > -25, points_array[:, 0] < 20)
        filtered_indices = np.logical_and(z_condition, x_condition)
        
        y_condition2 = np.logical_or(points_array[:, 1] <= -1.2, points_array[:, 1] >= 1.2)
        x_condition2 = np.logical_or(points_array[:, 0] <= -2.8, points_array[:, 0] >= 2)
        exclude_condition = np.logical_or(y_condition2, x_condition2)
        
        filtered_indices = np.logical_and(exclude_condition, filtered_indices)
        #non_ground_points = points_array[filtered_indices]

        # ground selection
        '''z_condition = np.logical_and(points_array[:, 2] > -1.9, points_array[:, 2] < -1.6)
        x_condition = np.logical_and(points_array[:, 0] > -4, points_array[:, 0] < 4)
        x_z_cond = np.logical_and(z_condition, x_condition)
        y_condition = np.logical_and(points_array[:, 1] > -3, points_array[:, 1] < 3)
        x_z_y_cond = np.logical_and(x_z_cond, y_condition)'''



        #ground_points = points_array[x_z_y_cond]

        '''if len(ground_points) > 100:
            ground_points_rand_indices = np.random.choice(ground_points.shape[0], 500, replace=True)
            ground_points = ground_points[ground_points_rand_indices]
            filtered_points = np.concatenate((ground_points, non_ground_points), axis=0)
        else:
            filtered_points = non_ground_points'''



        #filtered_indices = np.logical_or(filtered_indices, x_z_y_cond)
        filtered_points = points_array[filtered_indices]
        return filtered_points

    
    def filter_ground(self, points_array):
        z_condition = np.logical_and(points_array[:, 2] > 0, points_array[:, 2] < 1)
        filtered_points = points_array[z_condition]
        return filtered_points

    def odometry(self, point_cloud, odom_num):

        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the 3D data
        points = data[:, :-1]
        points = self.filter_points_fast(points)
        #points[:, 2] = -points[:, 2]

        # w dont use first 10 scans because they may have alot of noise
        if 10 <= odom_num:
            # get lidar transform of current step
            # lidar_tran = self.lidar_transform_list[odom_num]
            lidar_tran = point_cloud.transform
            # find ground truth position of current step and save it in a list
            self.pos_gt.append([lidar_tran.location.x, lidar_tran.location.y, lidar_tran.location.z])

        if odom_num == 10:
            self.T1 = lidar_tran.get_matrix()
            self.pos_est.append(self.pos_gt[0])

        elif odom_num == 11:
            self.T2 = lidar_tran.get_matrix()
            self.pos_est.append(self.pos_gt[1])
            
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

        elif 11<odom_num<self.scan_num:

            T_rel = self.registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            # T3 that transform scan3 to global coordinate is obtained by:
            #   --->  T3 = T2 * T_rel_32
            T3 = self.T2 @ self.T_rel_32

            self.pos_est.append(np.ndarray.tolist(T3[:3,-1]))

            self.T2 = T3
    
    def registration(self, data):
        # first we should prepare traget pcl
        target = o3d.geometry.PointCloud()
        target.points = o3d.utility.Vector3dVector(data)
        target_norm = None
        if self.voxel_size == 0:
            # without down sampling
            target_norm = target
            #target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=20))
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=10))
        else:
            # with down sampling
            target = target.voxel_down_sample(voxel_size= self.voxel_size)
            target_norm = target
            #target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.5, max_nn=8))
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.15, max_nn=10))

        # we should give an initial transform matrix to registration algorithm
        initial_transform = np.linalg.inv(self.T_rel_32)
        # we dont have a lot of changes in z axis so we assume the relative z = 0

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

    def plot_result(self, est, gt):
        print(gt)
        print(est)
        xg=gt[:self.scan_num-1,0]
        yg=gt[:self.scan_num-1,1]
        zg=gt[:self.scan_num-1,2]

        x=est[:self.scan_num-1,0]
        y=est[:self.scan_num-1,1]
        z=est[:self.scan_num-1,2]

        
        fig1, axs = plt.subplots(3)
        fig1.suptitle('(1 scan) AND (no subsampling) gt=green and est=red')
        fig1.subplots_adjust(hspace=0.7)
        axs[0].set_title('x-y')
        axs[0].plot(x, y, '-or')
        axs[0].plot(xg, yg, '-og')
        axs[0].set_xlabel('X')
        axs[0].set_ylabel('Y')

        axs[1].set_title('x-z')
        axs[1].plot(x, z, '-or')
        axs[1].plot(xg, zg, '-og')
        axs[1].set_xlabel('X')
        axs[1].set_ylabel('Z')
        axs[1].set_ylim([1,3])

        axs[2].set_title('y-z')
        axs[2].plot(y, z, '-or')
        axs[2].plot(yg, zg, '-og')
        axs[2].set_xlabel('Y')
        axs[2].set_ylabel('Z')
        axs[2].set_ylim([1,3])


        fig2 = plt.figure(figsize=(8, 6))
        ax = fig2.add_subplot(111, projection='3d')
        ax.scatter(xg, yg, zg, c='green', linewidths=0.1)
        ax.scatter(x, y, z, c='red',linewidths=0.1)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=30, azim=120)
        plt.show()
    

if __name__ == "__main__":

    try:
        OfflineLidarOdometry()
    except KeyboardInterrupt:
        print(' - Exited by user.')                                                      