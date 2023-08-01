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

from mechanism import Mechanism
from visualizer import visualizer
from imu_odometry import IMUOdometry

from util import destroy_queue

def main():
    # settings of simulation
    no_rendering = False
    don_flag = True
    delta = 0.1

    try:
        client = carla.Client('127.0.0.1', 3000)
        client.set_timeout(2.0)

        world = client.get_world()

        IMU_odom = IMUOdometry()
        #vehicle_transform = random.choice(world.get_map().get_spawn_points())
        mechanism_transform = world.get_map().get_spawn_points()[5]
        mechanism = Mechanism(world, client, mechanism_transform, IMU_odom)
        print('Mechanism is created')

        # Visualizer
        visual_msg_queue = Queue()
        quit = Value(c_bool, False)
        proc = Process(target =visualizer, args=(visual_msg_queue, quit))
        proc.daemon = True
        proc.start()

        # In case Matplotlib is not able to keep up the pace of the growing queue,
        # we have to limit the rate of the items being pushed into the queue
        visual_fps = 3
        last_ts = time.time()

        # Drive the car around and get sensor readings
        while True:
            #time.sleep(0.01)
            world.tick()
            
            
            # Limit the visualization frame-rate
            if time.time() - last_ts < 1. / visual_fps:
                continue

            frame = world.get_snapshot().frame

            if not IMU_odom.initialized:
                IMU_odom.initialize_with_ground_truth(mechanism.get_location(), mechanism.get_velocity(),
                 mechanism.get_T(), world.get_snapshot().timestamp)
                continue
            # Get sensor readings
            #sensors = mechanism.get_sensor_readings(frame)

            #print(IMU_odom.initialized)

            '''if sensors['imu'] is not None:
                IMU_odom.predict_state_with_imu(sensors['imu'])'''

            # timestamp for inserting a new item into the queue
            last_ts = time.time()

            # visual message
            visual_msg = dict()

            # Get ground truth vehicle location
            gt_location = mechanism.get_location()
            visual_msg['gt_traj'] = [gt_location.x, gt_location.y, gt_location.z] # round(x, 1)

            # Get gps reading

            visual_msg['est_traj'] = IMU_odom.get_location()          

            visual_msg_queue.put(visual_msg)

    finally:
        '''world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)'''
        print('Exiting visualizer')
        quit.value = True
        destroy_queue(visual_msg_queue)

        print('destroying the car object')
        mechanism.destroy()

        print('done')



if __name__ == '__main__':
    main()