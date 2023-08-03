import numpy as np
import math

import xml.etree.ElementTree as ET

class GNSS:
    def __init__(self, world, vehicle, blueprint_library, gnss_transform):
        self.world = world
        # GNSS parameters
        self.NOISE_STDDEV = 5e-5
        self.NOISE_BIAS = 1e-5
        self.GNSS_FREQ = 2

        # vehicle position obtained by GNSS
        self.x = None
        self.y = None
        self.z = None

        #gnss blueprint
        gnss_bp = self.generate_GNSS_bp(blueprint_library)

        # Attach GNSS to vehicle
        self.gnss = self.world.spawn_actor(
            blueprint=gnss_bp,
            transform=gnss_transform,
            attach_to=vehicle
        )

        # Reference latitude and longitude (GNSS)
        self.gnss_lat_ref, self.gnss_long_ref = self._get_latlon_ref()


    def generate_GNSS_bp(self, blueprint_library):
        """Generates a CARLA blueprint based on the script parameters"""
        # Sensor noise profile 
        NOISE_GNSS_ALT_BIAS = self.NOISE_BIAS
        NOISE_GNSS_ALT_STDDEV = self.NOISE_STDDEV
        NOISE_GNSS_LAT_BIAS = self.NOISE_BIAS
        NOISE_GNSS_LAT_STDDEV = self.NOISE_STDDEV
        NOISE_GNSS_LON_BIAS = self.NOISE_BIAS
        NOISE_GNSS_LON_STDDEV = self.NOISE_STDDEV

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
        gnss_bp.set_attribute('sensor_tick', str(1.0 / self.GNSS_FREQ))

        return gnss_bp
    
    def get_xyz(self, gnss_data):
        altitude = gnss_data.altitude
        latitude = gnss_data.latitude
        longitude = gnss_data.longitude

        return self.gnss_to_xyz(latitude, longitude, altitude)


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

        return x, y, altitude

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

    def destroy(self):
        self.gnss.destroy()