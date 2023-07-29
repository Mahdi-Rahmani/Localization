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
        # vehicle
        self.no_autopilot = True
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(self.vehicle_name)
        self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        self.vehicle.set_autopilot(self.no_autopilot)
        # GPS
        gnss_bp = blueprint_library.filter("sensor.other.gnss")[0]

        # Set sensors' noise
        gnss_bp.set_attribute('noise_alt_bias', str(NOISE_GNSS_ALT_BIAS))
        gnss_bp.set_attribute('noise_alt_stddev', str(NOISE_GNSS_ALT_STDDEV))
        gnss_bp.set_attribute('noise_lat_bias', str(NOISE_GNSS_LAT_BIAS))
        gnss_bp.set_attribute('noise_lat_stddev', str(NOISE_GNSS_LAT_STDDEV))
        gnss_bp.set_attribute('noise_lon_bias', str(NOISE_GNSS_LON_BIAS))
        gnss_bp.set_attribute('noise_lon_stddev', str(NOISE_GNSS_LON_STDDEV))

        # Sensor sampling frequency
        GNSS_FREQ = 1
        gnss_bp.set_attribute('sensor_tick', str(1.0 / GNSS_FREQ))

        self.gnss = world.spawn_actor(
            blueprint=gnss_bp,
            transform=carla.Transform(carla.Location(x=0, z=0)),
            attach_to=self.vehicle
        )

        self.actor_list = [self.vehicle, self.gnss]

        self.gnss_queue = Queue()

        # Hook sensor readings to callback methods
        weak_self = weakref.ref(self)
        self.gnss.listen(lambda data : Mechanism.sensor_callback(weak_self, data, self.gnss_queue))

        # Reference latitude and longitude (GNSS)
        self.gnss_lat_ref, self.gnss_long_ref = self._get_latlon_ref()

    def destroy(self):
        self.gnss.destroy()

        self.client.apply_batch([carla.command.DestroyActor(x) 
            for x in self.actor_list if x is not None])

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
        sensors = {'gnss': None}

        while not self.gnss_queue.empty():
            gnss_data = self.gnss_queue.get()
            #if gnss_data.frame == frame:

            alt = gnss_data.altitude
            lat = gnss_data.latitude
            long = gnss_data.longitude

            gps_xyz = self.gnss_to_xyz(lat, long, alt)
            sensors['gnss'] = gps_xyz
            
            self.gnss_queue.task_done()
            break

            #self.gnss_queue.task_done()

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