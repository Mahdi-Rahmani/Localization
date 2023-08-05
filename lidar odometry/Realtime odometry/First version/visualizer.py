
import matplotlib
matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

import time
import math

from numpy.core.shape_base import block

def add_to_traj(traj_x, traj_y, traj_z, x, y, z, max_len, min_dist=0.1):
    """Add new xyz position to the trajectory
    Two important considerations:
    1. remove the old points (sliding window) once the 'buffer' is full
    2. don't add the point if it is too close to the previous one

    :param traj: list of x locations
    :type traj: list
    :param traj: list of y locations
    :type traj: list
    :param traj: list of z locations
    :type traj: list
    :param x: x position
    :type x: double
    :param y: x position
    :type y: double
    :param z: z position
    :type z: double
    :param max_len: keep the last n positions
    :type max_len: int
    :param min_dist: min distance to consider adding the new point
    type min_dist: double
    """
    assert len(traj_x) == len(traj_y) and len(traj_y) == len(traj_z)
    # Calculate the distance between the current point and the last point
    # and skip if the distance is too small (e.g., vehicle is stationery)
    if len(traj_x) > 0 and \
        math.sqrt((traj_x[-1] - x)**2 + (traj_y[-1] - y) ** 2 + (traj_z[-1] - z) ** 2) < min_dist:
        return

    # Add new position to the trajectory
    traj_x.append(x)
    traj_y.append(y)
    traj_z.append(z)

    # Keep the lists within the maximum length
    # in-place modification
    if len(traj_x) > max_len:
        traj_x[:] = traj_x[-max_len:]
    if len(traj_y) > max_len:
        traj_y[:] = traj_y[-max_len:]
    if len(traj_z) > max_len:
        traj_z[:] = traj_z[-max_len:]    

def visualizer(visual_msg_queue, quit):
    # Setup layout
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor=(0.6, 0.6, 0.6))
    pose_plot = fig.add_subplot(111, projection='3d', facecolor=(1.0, 1.0, 1.0))
    # Set the title of the plot
    fig.suptitle('Lidar', fontsize=16)

    # Trajectory length limit
    # Would remove the oldest point as soon as the list reaches the maximum length (sliding window)
    max_traj_len = 50000

    # Keep vehicle trajectories
    gt_traj_x = []
    gt_traj_y = []
    gt_traj_z = []
    est_traj_x = []
    est_traj_y = []
    est_traj_z = []

    t = 0

    # Font size
    fontsize = 14

    # make sure the window is raised, but the script keeps going
    plt.show(block=False)

    while True: # not quit.value:
        if visual_msg_queue.empty():
            time.sleep(0.05)
            continue
        
        msg = visual_msg_queue.get()

        # Update trajectory
        updated_traj = False
        if msg.get('gt_traj') is not None:
            gt_pos = msg['gt_traj']
            add_to_traj(gt_traj_x, gt_traj_y, gt_traj_z, gt_pos[0], gt_pos[1], gt_pos[2], max_traj_len)
            updated_traj = True
        
        if msg.get('est_traj') is not None and len(msg.get('est_traj'))!=0:
            est_pos = msg['est_traj']
            add_to_traj(est_traj_x, est_traj_y, est_traj_z, est_pos[0], est_pos[1], est_pos[2]-2.15, max_traj_len)
            updated_traj = True

        # Visualize vehicle trajectory
        if updated_traj:
            # Clear previous plot
            pose_plot.cla()

            # Update plot
            pose_plot.plot(gt_traj_x, gt_traj_y, gt_traj_z, color='green', linestyle='solid', label='GT')
            pose_plot.plot(est_traj_x, est_traj_y, est_traj_z, color='red', linestyle='solid', label='est')
            pose_plot.legend(fontsize=fontsize)
        
        # flush any pending GUI events, re-painting the screen if needed
        fig.canvas.flush_events()
        t += 1
        
        if not quit.value:
            fig.canvas.draw_idle()
        else:
            plt.close('all')
            print('quiting visualizer loop')
            break
        