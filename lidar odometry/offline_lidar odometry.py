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
        self.scan_num = 500
        self.counter = 0
        self.pcls_list = []
        self.lidar_transform_list = []
        self.T1 = None
        self.T2 = None
        self.T_rel_32 = None
        self.pos_gt = []
        self.pos_est = []


    def create_environment(self):
        client = carla.Client(host, port)
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
            vehicle_transform = random.choice(world.get_map().get_spawn_points())
            self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
            self.vehicle.set_autopilot(no_autopilot)

            lidar_bp = self.generate_lidar_bp(world, blueprint_library, delta)
            x = 0.0
            y = 0.0
            z = 0.0
            user_offset = carla.Location(x, y, z)
            lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)

            self.lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
            self.lidar.listen(lambda data: self.lidar_callback(data))

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
            self.locallization()
            world.apply_settings(original_settings)
            traffic_manager.set_synchronous_mode(False)

            vehicle.destroy()
            lidar.destroy()

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
        self.counter +=1
        if self.counter == self.scan_num:
            self.don_flag = False
        self.pcls_list.append(point_cloud)
        self.lidar_transform_list.append(self.lidar.get_transform())

    def locallization(self):
        odom_num = 0
        for pcl in my_pcls:
            odom_num += 1
            self.odometry(pcl,odom_num)
        
    def odometry(self, point_cloud, odom_num):

        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Isolate the 3D data
        points = data[:, :-1]

        # get lidar transform of current step
        lidar_tran = self.lidar_transform[counter]
        # find ground truth position of current step and save it in a list
        self.pos_gt.append([lidar_tran.location.x, lidar_tran.location.y, lidar_tran.location.z])

        if odom_num == 1:
            self.T1 = lidar_tran.get_matrix()
            self.pos_est.append(self.pos_gt[0])

        elif odom_num == 2:
            self.T2 = lidar_tran.get_matrix()
            self.pos_est.append(self.pos_gt[1])
            
            # We know that T2 = T1 * T_rel_21 so we can compute T_rel_21:
            T_rel_21 = np.linalg.inv(self.T1) @ T2

            # also from constant velocity assumption we have:
            #   -->  T_rel_32 = T_rel_21
            self.T_rel_32 = T_rel_21

        elif 2<odom_num<self.scan_num:

            T_rel = registration(points)
            self.T_rel_32 = np.linalg.inv(T_rel)

            T3 = T2 * self.T_rel_32

            self.pos_est.append(np.ndarray.tolist(T3[:3,-1]))

            self.T2 = T3
        




    

