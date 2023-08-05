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
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import threading
import queue

class OnlineOdometry():
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
        self.scan_num = 150
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
        # self.voxel_size = 4
        # self.threshold = 2
        # without down sampling
        self.voxel_size = 0.5
        self.threshold = 1.3

        # plot
        self.ground_truth_data = np.empty((0, 3))
        self.sensor_data = np.empty((0, 3))
        # Create a thread-safe queue to store sensor data
        self.data_queue = queue.Queue()

        self.plot_result()

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

            self.lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            self.lidar.listen(lambda data: self.lidar_callback(data))

            self.set_all_traffic_lights_to_green(world)

            frame = 0
            dt0 = datetime.now()

            while self.don_flag:
                # # This can fix Open3D jittering issues:
                time.sleep(0.005)
                world.tick()

                process_time = datetime.now() - dt0
                sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
                sys.stdout.flush()
                dt0 = datetime.now()
                frame += 1
        finally:
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

    def lidar_callback(self, point_cloud):
        """ This function is called after each lidar scan and we save
        pcls and lidar_transform in a list. lidar transform contains groud truth """
        self.counter +=1
        self.odometry(point_cloud, self.counter)

    def odometry(self, point_cloud, odom_num):

        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the 3D data
        points = data[:, :-1]
        points = self.filter_points_fast(points)
        #points[:, 2] = -points[:, 2]

        # w dont use first 10 scans because they may have alot of noise
        if 8 <= odom_num:
            # get lidar transform of current step
            # lidar_tran = self.lidar_transform_list[odom_num]
            lidar_tran = point_cloud.transform
            # find ground truth position of current step and save it in a list
            new_ground_truth = np.array([lidar_tran.location.x, lidar_tran.location.y, lidar_tran.location.z])
            self.ground_truth_data = np.vstack((self.ground_truth_data, new_ground_truth))
        
        if odom_num == 8:
            self.T1 = lidar_tran.get_matrix()
            # self.sensor_data = np.vstack((self.sensor_data, new_ground_truth))
            self.data_queue.put(new_ground_truth)

        elif odom_num == 9:
            self.T2 = lidar_tran.get_matrix()
            #self.sensor_data = np.vstack((self.sensor_data, new_ground_truth))
            self.data_queue.put(new_ground_truth)
            
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

        elif 9 < odom_num:

            T_rel = self.registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            # T3 that transform scan3 to global coordinate is obtained by:
            #   --->  T3 = T2 * T_rel_32
            T3 = self.T2 @ self.T_rel_32

            new_sensor_data = np.array(np.ndarray.tolist(T3[:3,-1]))
            self.data_queue.put(new_sensor_data)
            #self.sensor_data = np.vstack((self.sensor_data, new_sensor_data))

            self.T2 = T3
    
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
            #target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.5, max_nn=8))
            target_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.15, max_nn=10))


        # we should give an initial transform matrix to registration algorithm
        initial_transform = np.linalg.inv(self.T_rel_32)

        # point to plane ICP registraion algorithm
        reg_p2l = o3d.pipelines.registration.registration_icp(
        self.source, target_norm, self.threshold, init =initial_transform,
        estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0000001,
                                                                        relative_rmse=0.0000001,
                                                                        max_iteration=20))
        # The source we used here is a target for next iteration
        self.source = target

        return reg_p2l.transformation        

    '''def sampling(self, pcd):
        # Estimate the ground plane using RANSAC
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=700)

        # Extract ground points
        ground_points = pcd.select_by_index(inliers)

        # Remove ground points from the point cloud
        non_ground_points = pcd.select_by_index(inliers, invert=True)

        non_ground_points = non_ground_points.voxel_down_sample(voxel_size= self.voxel_size)

        return non_ground_points'''


    def plot_result(self):
        # Initialize the figure and 3D self.axis
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')

        ani = FuncAnimation(fig, self.update_plot, interval=1000)  # Update plot every 1000 ms (1 second)

        # Start the simulation in a separate thread
        simulation_thread = threading.Thread(target=self.create_environment)
        simulation_thread.daemon = True
        simulation_thread.start()

        # Show the real-time plot
        plt.show()

   # Function to update the plot in real-time
    def update_plot(self, frame):

        # Fetch new sensor data
        try:
            new_sensor_data = self.data_queue.get_nowait()
            self.sensor_data = np.vstack((self.sensor_data, new_sensor_data))
            print(new_sensor_data)
        except queue.Empty:
            pass
        
        # Clear the previous plot
        self.ax.cla()

        # Plot ground truth and sensor data
        self.ax.scatter(self.ground_truth_data[:, 0], self.ground_truth_data[:, 1], self.ground_truth_data[:, 2], c='b', label='Ground Truth')
        self.ax.scatter(self.sensor_data[:, 0], self.sensor_data[:, 1], self.sensor_data[:, 2], c='r', label='Sensor Data')

        # Connect the dots in sequence for both ground truth and sensor data
        self.ax.plot(self.ground_truth_data[:, 0], self.ground_truth_data[:, 1], self.ground_truth_data[:, 2], c='b')
        self.ax.plot(self.sensor_data[:, 0], self.sensor_data[:, 1], self.sensor_data[:, 2], c='r')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend()


if __name__ == "__main__":

    try:
        OnlineOdometry()
    except KeyboardInterrupt:
        print(' - Exited by user.')                                                      