#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from lidar_odom.msg import location
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


class RealTime3DPlotNode:
    def __init__(self):
        rospy.init_node('real_time_3d_plot_node')
        self.localization_sub = rospy.Subscriber('localization', location, self.localization_callback)
        self.x_vals = []
        self.y_vals = []
        self.z_vals = []

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
        self.z_vals.append(msg.z)

    def update_plot(self, frame):
        self.ax.cla()
        self.ax.plot(self.x_vals, self.y_vals, self.z_vals)

        # Set z-axis limits manually to ensure visibility
        if self.z_vals:
            self.ax.set_zlim(min(self.z_vals), max(self.z_vals))
        
        # Add this line to return the plotted line as a sequence of Artist objects
        return [self.ax]

if __name__ == '__main__':
    try:
        plot_node = RealTime3DPlotNode()
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


'''class PathMonitor:
    
    def __init__(self) -> None:
        
        rospy.init_node("monitor" , anonymous=False)
        
        self.path = Path()
        self.odom_subscriber = rospy.Subscriber("localization" , location , callback=self.odom_callback)
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)
        
        
    def odom_callback(self, msg : Odometry):

        x = msg.x
        y = msg.y
        z = msg.z

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)
        
        
if __name__ == "__main__":
    path_monitor = PathMonitor()
    
    rospy.spin()'''