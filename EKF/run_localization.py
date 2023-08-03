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

import random
import cv2
import time

from multiprocessing import Queue, Value, Process
from ctypes import c_bool

from vehicle import Vehicle
from lidar import LIDAR
from gnss import GNSS
from imu import IMU

from extended_kalman_filter import ExtendedKalmanFilter
from visualizer import visualizer

from util import destroy_queue

def main():
    try:
        # prepare_environment
        # connect to server
        client = carla.Client('127.0.0.1', 3000)
        client.set_timeout(2.0)

        # create world
        world = client.get_world()

        # create vehicle and its sensors
        blueprint_library = world.get_blueprint_library()
        # create vehicle
        vehicle_obj = Vehicle(world, blueprint_library)
        # cretae sensors
        lidar_obj = LIDAR(world, vehicle_obj.vehicle, blueprint_library, carla.Transform(carla.Location(x=-0.5, z=1.8)))
        gnss_obj = GNSS(world, vehicle_obj.vehicle, blueprint_library, carla.Transform(carla.Location(x=0, z=0)))
        imu_obj = IMU(world, vehicle_obj.vehicle, blueprint_library, carla.Transform(carla.Location(x=0, z=0)))
        actor_list = [vehicle_obj.vehicle, lidar_obj.lidar, gnss_obj.gnss, imu_obj.imu]
        print('Vehicle and Sensors are created.')

        # EKF
        ekf = ExtendedKalmanFilter(lidar_obj, gnss_obj, imu_obj)

        # Visualizer
        visual_msg_queue = Queue()
        quit = Value(c_bool, False)
        proc = Process(target =visualizer, args=(visual_msg_queue, quit))
        proc.daemon = True
        proc.start()

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.1
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)
        '''settings.fixed_delta_seconds = 0.03
        settings.synchronous_mode = True
        settings.no_rendering_mode = False
        world.apply_settings(settings)'''
        # In case Matplotlib is not able to keep up the pace of the growing queue,
        # we have to limit the rate of the items being pushed into the queue
        visual_fps = 3
        last_ts = time.time()

        # Drive the car around and get sensor readings
        while True:
            time.sleep(0.005)
            world.tick()
            #frame = world.get_snapshot().frame
            
            # EKF initialization
            # Don't run anything else before EKF is initialized
            if ekf.initialized:
                vehicle_obj.set_autopilot_status(True)
            else:
                continue

            # Limit the visualization frame-rate
            if time.time() - last_ts < 1. / visual_fps:
                continue
            
            # timestamp for inserting a new item into the queue
            last_ts = time.time()

            # visual message
            visual_msg = dict()

            # Get ground truth vehicle location
            gt_location = vehicle_obj.get_location()
            visual_msg['gt_traj'] = [gt_location.x, gt_location.y, gt_location.z] # round(x, 1)

            # Get estimated location
            visual_msg['est_traj'] = ekf.get_location()          

            visual_msg_queue.put(visual_msg)

    finally:
        print('Exiting visualizer')
        quit.value = True
        destroy_queue(visual_msg_queue)

        print('destroying the car object')
        lidar_obj.destroy()
        gnss_obj.destroy()
        imu_obj.destroy()
        vehicle_obj.destroy()

        client.apply_batch([carla.command.DestroyActor(x) 
            for x in actor_list if x is not None])

        world.apply_settings(original_settings)
        print('done')


if __name__ == '__main__':
    main()