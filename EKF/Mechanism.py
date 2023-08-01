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
from util import destroy_queue

class Gnss:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Mechanism:
    def __init__(self, world, client, vehicle_transform):
        self.world = world
        self.client = client

        # Initialize the vehicle and the sensors
        # vehicle
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(self.vehicle_name)
        self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        # sensors
        lidar_bp = self.generate_lidar_bp(blueprint_library)
        imu_bp = self.generate_IMU_bp(blueprint_library)
        gnss_bp = self.generate_GNSS_bp(blueprint_library)

        # Attach sensors to vehicle
        self.lidar = world.spawn_actor(
            blueprint=lidar_bp,
            transform=carla.Transform(carla.Location(x=-0.5, z=1.8)),
            attach_to=self.vehicle
        )
        self.imu = world.spawn_actor(
            blueprint=imu_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )
        self.gnss = world.spawn_actor(
            blueprint=gnss_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )

        self.actor_list = [self.vehicle, self.lidar, self.imu, self.gnss]

        # vehicle start moving
        self.vehicle.set_autopilot(True)

        # Keep the sensor readings from the callback to queues
        self.lidar_queue = Queue()
        self.imu_queue = Queue()
        self.gnss_queue = Queue()

        # Hook sensor readings to callback methods
        weak_self = weakref.ref(self)
        self.lidar.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.lidar_queue))
        self.imu.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.imu_queue))
        self.gnss.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.gnss_queue))

        # Reference latitude and longitude (GNSS)
        self.gnss_lat_ref, self.gnss_long_ref = self._get_latlon_ref()

    def generate_lidar_bp(self, blueprint_library):

        """Generates a CARLA blueprint based on the script parameters"""
        # parameters
        no_noise = False
        show_axis = False
        upper_fov = 15.0
        lower_fov = -25.0
        channels = 64.0
        lidar_range = 100.0
        points_per_second = 1000000
        delta = 0.1
        # find blueprint from library
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')

        # set parameters
        if no_noise:
            lidar_bp.set_attribute('dropoff_general_rate', '0.0')
            lidar_bp.set_attribute('dropoff_intensity_limit', '1.0')
            lidar_bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            lidar_bp.set_attribute('noise_stddev', '0.02')

        lidar_bp.set_attribute('upper_fov', str(upper_fov))
        lidar_bp.set_attribute('lower_fov', str(lower_fov))
        lidar_bp.set_attribute('channels', str(channels))
        lidar_bp.set_attribute('range', str(lidar_range))

        # Sensor sampling frequency
        lidar_bp.set_attribute('rotation_frequency', str(1.0 / delta))

        lidar_bp.set_attribute('points_per_second', str(points_per_second))
        return lidar_bp

    def generate_IMU_bp(self, blueprint_library):

        """Generates a CARLA blueprint based on the script parameters"""
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

        # initilize IMU sensor
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

        return imu_bp

    def generate_GNSS_bp(self, blueprint_library):

        """Generates a CARLA blueprint based on the script parameters"""
        # Sensor noise profile 
        NOISE_STDDEV = 5e-5
        NOISE_BIAS = 1e-5
        NOISE_GNSS_ALT_BIAS = NOISE_BIAS
        NOISE_GNSS_ALT_STDDEV = NOISE_STDDEV
        NOISE_GNSS_LAT_BIAS = NOISE_BIAS
        NOISE_GNSS_LAT_STDDEV = NOISE_STDDEV
        NOISE_GNSS_LON_BIAS = NOISE_BIAS
        NOISE_GNSS_LON_STDDEV = NOISE_STDDEV

        # Initialize the vehicle and the sensors
        gnss_bp = blueprint_library.filter("sensor.other.gnss")[0]

        # Set sensors' noise
        gnss_bp.set_attribute('noise_alt_bias', str(NOISE_GNSS_ALT_BIAS))
        gnss_bp.set_attribute('noise_alt_stddev', str(NOISE_GNSS_ALT_STDDEV))
        gnss_bp.set_attribute('noise_lat_bias', str(NOISE_GNSS_LAT_BIAS))
        gnss_bp.set_attribute('noise_lat_stddev', str(NOISE_GNSS_LAT_STDDEV))
        gnss_bp.set_attribute('noise_lon_bias', str(NOISE_GNSS_LON_BIAS))
        gnss_bp.set_attribute('noise_lon_stddev', str(NOISE_GNSS_LON_STDDEV))

        # Sensor sampling frequency
        GNSS_FREQ = 5
        gnss_bp.set_attribute('sensor_tick', str(1.0 / GNSS_FREQ))

        return gnss_bp

    def destroy(self):
        self.lidar.destroy()
        self.imu.destroy()
        self.gnss.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])
        
        destroy_queue(self.lidar_queue)
        destroy_queue(self.imu_queue)
        destroy_queue(self.gnss_queue)


    def get_location(self):
        return self.vehicle.get_location()    
        
    @staticmethod
    def sensor_callback(weak_self, data, queue):
        self = weak_self()
        if not self:
            return
        queue.put(data)

    def get_sensor_readings(self, frame):
        """Return a dict containing the sensor readings
        at the particular frame

        :param frame: unique frame at the current world frame
        :type frame: int
        """
        sensors = {'lidar': None,
                   'imu': None,
                   'gnss': None}

        while not self.lidar_queue.empty():
            # lidar data is point cloud
            lidar_data = self.lidar_queue.get()
            #print("frame: ", frame, "lidar frame: ", lidar_data.frame)
            if lidar_data.frame+1 == frame:
                # the output of lidar odometry is [x, y, z] which is estimated
                data = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                data = np.reshape(data, (int(data.shape[0] / 4), 4))
                pose = self.lidar_odom.odometry(lidar_data)
                sensors['lidar'] = pose
                self.lidar_queue.task_done()
                break

            self.lidar_queue.task_done()
    
        while not self.imu_queue.empty():
            imu_data = self.imu_queue.get()
        
            if imu_data.frame == frame:
                sensors['imu'] = imu_data

                self.imu_queue.task_done()
                break

            self.imu_queue.task_done()

        while not self.gnss_queue.empty():
            gnss_data = self.gnss_queue.get()

            if gnss_data.frame == frame:

                alt = gnss_data.altitude
                lat = gnss_data.latitude
                long = gnss_data.longitude

                gps_xyz = self.gnss_to_xyz(lat, long, alt)

                sensors['gnss'] = gps_xyz

                self.gnss_queue.task_done()
                break

            self.gnss_queue.task_done()
        

        return sensors

    def gnss_to_xyz(self, latitude, longitude, altitude):
        """Creates Location from GPS (latitude, longitude, altitude).
        This is the inverse of the _location_to_gps method found in
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        
        Modified from:
        https://github.com/erdos-project/pylot/blob/master/pylot/utils.py
        """
        EARTH_RADIUS_EQUA = 6378137.0

        scale = math.cos(self.gnss_lat_ref * math.pi / 180.0)
        basex = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * self.gnss_long_ref
        basey = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + self.gnss_lat_ref) * math.pi / 360.0))

        x = scale * math.pi * EARTH_RADIUS_EQUA / 180.0 * longitude - basex
        y = scale * EARTH_RADIUS_EQUA * math.log(
            math.tan((90.0 + latitude) * math.pi / 360.0)) - basey

        # This wasn't in the original method, but seems to be necessary.
        y *= -1

        return Gnss(x, y, altitude)

    def _get_latlon_ref(self):
        """
        Convert from waypoints world coordinates to CARLA GPS coordinates
        :return: tuple with lat and lon coordinates
        https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
        """
        xodr = self.world.get_map().to_opendrive()
        tree = ET.ElementTree(ET.fromstring(xodr))

        # default reference
        lat_ref = 42.0
        lon_ref = 2.0

        for opendrive in tree.iter("OpenDRIVE"):
            for header in opendrive.iter("header"):
                for georef in header.iter("geoReference"):
                    if georef.text:
                        str_list = georef.text.split(' ')
                        for item in str_list:
                            if '+lat_0' in item:
                                lat_ref = float(item.split('=')[1])
                            if '+lon_0' in item:
                                lon_ref = float(item.split('=')[1])
        return lat_ref, lon_ref