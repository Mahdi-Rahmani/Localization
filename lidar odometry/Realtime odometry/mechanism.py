
'''
    The vehicle class contains vehicle and its attached sensors. We create 
    these blueprints here and also implement their callbacks.
    According to sensor frequency when we get data from that sensor then 
    sensor's callback is called and data is pushed into a queue.

    Author: Mahdi Rahmani
'''
import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import numpy as np
import math

import weakref
from queue import Queue
from collections import OrderedDict
from util import destroy_queue

from lidar_odometry import LidarOdometry
import open3d as o3d

class Mechanism:

    def __init__(self, world, client, vehicle_transform):
        self.world = world
        self.client = client
        
        # settings of vehicle and sensors
        self.no_noise = True
        self.no_autopilot = True
        self.show_axis = False
        self.upper_fov = 15.0
        self.lower_fov = -25.0
        self.channels = 64.0
        self.lidar_range = 100.0
        self.points_per_second = 1000000

        self.delta = 0.1 

        # bluprints those added to carla
        # vehicle
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(self.vehicle_name)
        self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        self.vehicle.set_autopilot(self.no_autopilot)
        # lidar
        lidar_bp = self.generate_lidar_bp(world, blueprint_library, self.delta)
        x = 0.0
        y = 0.0
        z = 0.0
        user_offset = carla.Location(x, y, z)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + user_offset)
        self.lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)

        self.actor_list = [self.vehicle, self.lidar]

        # Keep the sensor readings from the callback to queue
        self.lidar_queue = Queue()

        # Hook sensor reading to callback methods
        weak_self = weakref.ref(self)
        self.lidar_odom = LidarOdometry()
        self.lidar.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.lidar_queue))


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

    @staticmethod
    def sensor_callback(weak_self, data, queue):
        self = weak_self()
        if not self:
            return
        queue.put(data)

    def get_location(self):
        return self.vehicle.get_location()    
        

    def get_sensor_readings(self, frame):
        """Return a dict containing the sensor readings
            at the particular frame

            :param frame: unique frame at the current world frame
            :type frame: int
        """
        sensors = {'lidar': None}
        while not self.lidar_queue.empty():
            # lidar data is point cloud
            lidar_data = self.lidar_queue.get()
            #print("frame: ", frame, "lidar frame: ", lidar_data.frame)
            if lidar_data.frame+1 == frame:
                #print("KKKKKKKKKKKKKKKKKKKKKKKKK")
                # the output of lidar odometry is [x, y, z] which is estimated
                data = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                data = np.reshape(data, (int(data.shape[0] / 4), 4))
                pose = self.lidar_odom.odometry(lidar_data)
                sensors['lidar'] = pose
                #sensors['lidar'] = [0, 1, 2]
                self.lidar_queue.task_done()
                break

            self.lidar_queue.task_done()
        #print(sensors)
        return sensors
         

    def destroy(self):
        self.lidar.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])
        
        destroy_queue(self.lidar_queue)