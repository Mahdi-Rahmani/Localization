# importing libraries
import glob
import os
import sys

import time
import random
import numpy as np
import math

# libraries related to visualization
from multiprocessing import Process, Queue
from util import destroy_queue
from ctypes import c_bool

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

# Import my other python files
from mechanism import Mechanism
from visualizer import visualizer

class OfflineLidarOdometry():
    def __init__(self):
        # carla host and port
        self.host = '127.0.0.1'
        self.port = 3000

        # settings of simulation
        self.delta = 0.1 
        self.no_rendering = False
        self.don_flag = True

        # visualization parameters
        # visual message
        self.visual_msg = dict()

        self.create_environment()

    def create_environment(self):

        try:
            # create client and connect it to server
            client = carla.Client(self.host, self.port)
            client.set_timeout(2.0)

            # Create World and Mechanism
            world = client.get_world()
            #vehicle_transform = random.choice(world.get_map().get_spawn_points())
            mechanism_transform = world.get_map().get_spawn_points()[5]
            mechanism = Mechanism(world, client, mechanism_transform)
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

            while True:
                # # This can fix Open3D jittering issues:
                world.tick()
                frame = world.get_snapshot().frame

                # Get sensor readings
                sensors = mechanism.get_sensor_readings(frame)

                # Limit the visualization frame-rate
                if time.time() - last_ts < 1. / visual_fps:
                    continue

                # timestamp for inserting a new item into the queue
                last_ts = time.time()

                 # visual message
                visual_msg = dict()

                # Get ground truth vehicle location
                gt_location = mechanism.get_location()
                visual_msg['gt_traj'] = [gt_location.x, gt_location.y, gt_location.z] # round(x, 1)

                # Get estimated location
                if sensors['lidar'] is not None:
                    visual_msg['est_traj'] = sensors

                visual_msg_queue.put(visual_msg)

                
        finally:
            print('Exiting visualizer')
            quit.value = True
            destroy_queue(visual_msg_queue)

            print('destroying the Mechanism object')
            mechanism.destroy()

            print('done')       


if __name__ == "__main__":

    try:
        OfflineLidarOdometry()
    except KeyboardInterrupt:
        print(' - Exited by user.')                                                      