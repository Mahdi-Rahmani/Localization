#!/usr/bin/python3

from lidar_odom.msg import location
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import rospy


class RealTime3DPlotNode:
    def __init__(self):
        rospy.init_node('real_time_3d_plot_node')
        self.localization_sub = rospy.Subscriber('localization', location, self.localization_callback)
        self.x_vals = []
        self.y_vals = []
        self.z_vals = []
        self.ground_truth_x = []
        self.ground_truth_y = []
        self.ground_truth_z = []

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Real-Time 3D Localization Plot')

        self.anim = FuncAnimation(
            self.fig,
            self.update_plot,
            frames=100,
            blit=True,
            interval=100,
            cache_frame_data=False
        )

    def localization_callback(self, msg):
        self.x_vals.append(msg.x)
        self.y_vals.append(msg.y)
        self.z_vals.append(0.0)
        #self.ground_truth_x.append(msg.x_gt)
        #self.ground_truth_y.append(msg.y_gt)
        #self.ground_truth_z.append(msg.z_gt)


    def update_plot(self, frame):
        self.ax.cla()
        self.ax.plot(self.x_vals, self.y_vals, self.z_vals)

        # Plot estimated data in red
        self.ax.plot(self.x_vals, self.y_vals, self.z_vals, color='red', label='Estimated')

        # Plot ground truth data in green
        #self.ax.plot(self.ground_truth_x, self.ground_truth_y, self.ground_truth_z, color='green', label='Ground Truth')

        # Set z-axis limits manually to ensure visibility
        #all_z_vals = self.z_vals + self.ground_truth_z
        #if all_z_vals:
        #    self.ax.set_zlim(min(all_z_vals), max(all_z_vals))
        self.ax.axis('equal')
        #self.ax.view_init(elev=90, azim=0)
        # Add legend
        self.ax.legend()
        
        # Add this line to return the plotted line as a sequence of Artist objects
        return [self.ax]

if __name__ == '__main__':
    try:
        plot_node = RealTime3DPlotNode()
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
