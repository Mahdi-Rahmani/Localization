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

import xml.etree.ElementTree as ET

from queue import Queue
from collections import OrderedDict
from imu_odometry import IMUOdometry

from util import destroy_queue


class Mechanism:
    def __init__(self, world, client, vehicle_transform, IMU_odom):
        self.world = world
        self.client = client

        # Sensor noise profile 
        NOISE_STDDEV = 5e-5
        NOISE_BIAS = 1e-5
        NOISE_IMU_ACC_X_STDDEV = NOISE_STDDEV
        NOISE_IMU_ACC_Y_STDDEV = NOISE_STDDEV
        NOISE_IMU_ACC_Z_STDDEV = NOISE_STDDEV
        NOISE_IMU_GYRO_X_BIAS = NOISE_BIAS
        NOISE_IMU_GYRO_X_STDDEV = NOISE_STDDEV
        NOISE_IMU_GYRO_Y_BIAS = NOISE_BIAS
        NOISE_IMU_GYRO_Y_STDDEV = NOISE_STDDEV
        NOISE_IMU_GYRO_Z_BIAS = NOISE_BIAS
        NOISE_IMU_GYRO_Z_STDDEV = NOISE_STDDEV
        
        # Initialize the vehicle and the sensors
        # vehicle
        self.no_autopilot = True
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(self.vehicle_name)
        self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        self.vehicle.set_autopilot(self.no_autopilot)
        # IMU
        imu_bp = blueprint_library.filter("sensor.other.imu")[0]

        # Set sensors' noise
        imu_bp.set_attribute('noise_accel_stddev_x', str(NOISE_IMU_ACC_X_STDDEV))
        imu_bp.set_attribute('noise_accel_stddev_y', str(NOISE_IMU_ACC_Y_STDDEV))
        imu_bp.set_attribute('noise_accel_stddev_z', str(NOISE_IMU_ACC_Z_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_x', str(NOISE_IMU_GYRO_X_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_y', str(NOISE_IMU_GYRO_Y_STDDEV))
        imu_bp.set_attribute('noise_gyro_stddev_z', str(NOISE_IMU_GYRO_Z_STDDEV))
        imu_bp.set_attribute('noise_gyro_bias_x', str(NOISE_IMU_GYRO_X_BIAS))
        imu_bp.set_attribute('noise_gyro_bias_y', str(NOISE_IMU_GYRO_Y_BIAS))
        imu_bp.set_attribute('noise_gyro_bias_z', str(NOISE_IMU_GYRO_Z_BIAS))

        # Sensor sampling frequency
        IMU_FREQ = 200
        imu_bp.set_attribute('sensor_tick', str(1.0 / IMU_FREQ))

        self.imu = world.spawn_actor(
            blueprint=imu_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )

        self.actor_list = [self.vehicle, self.imu]

        self.imu_queue = Queue()

        self.IMU_odom = IMU_odom
        # Hook sensor readings to callback methods
        weak_self = weakref.ref(self)
        self.imu.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.imu_queue))

        


    def destroy(self):
        self.imu.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])

        destroy_queue(self.imu_queue)


    def get_location(self):
        return self.vehicle.get_location() 

    def get_T(self):
        return self.vehicle.get_transform()
    
    def get_velocity(self):
        return self.vehicle.get_velocity()
        
    @staticmethod
    def sensor_callback(weak_self, data, queue):
        self = weak_self()
        if not self:
            return
        #queue.put(data)
        if self.IMU_odom.initialized:
            self.IMU_odom.predict_state_with_imu(data)
        else:
            self.IMU_odom.set_last_ts(data)

    '''def get_sensor_readings(self, frame):
        """Return a dict containing the sensor readings
        at the particular frame

        :param frame: unique frame at the current world frame
        :type frame: int
        """
        sensors = {'imu': None}

        while not self.imu_queue.empty():
            imu_data = self.imu_queue.get()
            #print("imu_data.frame", imu_data.frame, "frame", frame)
            if imu_data.frame == frame:
                sensors['imu'] = imu_data

                self.imu_queue.task_done()
                break

            self.imu_queue.task_done()

        self.imu_queue.task_done()
        return sensors'''