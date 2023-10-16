# Lidar based self localization in autonomous vehicles
<p align="center">
  <a href="https://github.com/Mahdi-Rahmani/Localization/blob/main/pics/res1.png">
    <img src="https://github.com/Mahdi-Rahmani/Localization/blob/main/pics/res1.png" alt="CE-AUT MAP">
  </a>
</p>

- **By**: Mahdi Rahmani  

- **Supervisor**: Dr. Mahdi Javanmardi

------------------

This is my bachelor's project. I will explain my work step by step here. In this project my purpose is to create a robot that can be self localized in my college environment. For doing this job, after I getting to know some basics of localization I should develop a code for implementing some localization algorithm and also handling some sensor fusion if it is needed. Before testing my code on a real agent in a real environment I should examin it in a simulator, so I choose Carla simulator for this purpose. As you have noticed so far, this project has two phases:
- Simulation phase
- Experimental phase

In the following, we will explain about each of them.

## Steps
First, after a little research, I had to choose the subject of my project and prepare the proposal file. To check the proposal file in Farsi, you can go [here](https://github.com/Mahdi-Rahmani/Localization/tree/main/proposal) 
According to the time table in my proposal I should do this works in sequence: 

1- Finding related resources and articles and study of them 

2- Getting familiar with softwares like carla simulator 

3- Localization with Lidar sensor in carla 

4- Increasing the accuracy of localization that we reached in the previous step (Use other sensors and fusion)

5- Creating a robot and test my algorithm in college environment 

6- Preparing my final report  
 
First phase, finding related resourses, is done and you can see some related articles in [articles directory](https://github.com/Mahdi-Rahmani/Localization/tree/main/articles) and some good books in [books directory](https://github.com/Mahdi-Rahmani/Localization/tree/main/books). For second phase, I explain completely in next section. 

## Softwares
In this research, we used different software. In the simulation part, we used the Carla simulator and in the practical part, we used the ROS operating system, as well as the packages and software of Ouster company to work with the lidar sensor.

### 1) Carla simulator 
This simulator was created for the development of self-driving cars and provides us with features such as choosing different urban and rural environments, creating different sensors on the car, creating weather changes, etc.

Also, this software uses client-server architecture. In fact, with the Python codes we write on the client side, we can apply the changes on the server side, which is our simulator. 

You can refer to the [link](https://carla.readthedocs.io/en/0.9.12/start_quickstart/) to install this software. In this research, we used version 0.9.12 and the operating system used was Windows 11.

This simulator requires a good graphics card and RAM for optimal performance. The hardware specifications of my laptop in this research are as follows:
- **CPU**:  Core(TM) i7-12650H
- **RAM**: 16GB DDR5
- **Hard disk**: 500GB SSD
- **GPU**: NVIDIA GeForce RTX 3070 Laptop GPU

But you can refer to the [link](https://carla.readthedocs.io/en/0.9.12/start_quickstart/) to find out more details about the hardware and software requirements.

### 2) ROS
ROS, or Robot Operating System, is an open-source middleware framework used for developing and controlling robots. Despite its name, ROS is not an actual operating system but rather a collection of software libraries and tools that facilitate various aspects of robotics development. 

In this research, we used the Ros-noetic version. For this purpose, I did a dual boot with Ubuntu version 20.04 and installed this software on this operating system. You can refer to the [link](https://wiki.ros.org/noetic/Installation/Ubuntu) to complete the installation process.
Our goal in this research was to be able to perform localization task in real time according to the data coming from the lidar sensor. ROS made this issue easier for us.

Also, the two tools **ROS-bag** and **RViz** helped us a lot for this part. We used ROS-bag to collect and record as well as replay the data obtained from the lidar sensor, as well as RViz to display the point cloud data and the route drawn with the help of localization.

### 3) Ouster related packages and software
In order to be able to connect the lidar sensor to the laptop and perform the data collection, it is necessary to install a series of packages first. Also, after downloading and installing them, it is necessary to make some settings according to the user-manual of the sensor so that the data packets related to lidar can be received by the system.

Among the most important software and packages for installation are:
- ouster SDK
- ouster-ros package
- ouster studio software

**Ouster lidar config:**  We use Ouster lidar with 64 layers and 10Hz frequency and range of 100 meters and it can give us 1 million points in each scan. 


In [ROS & Real data](https://github.com/Mahdi-Rahmani/Localization/tree/main/ROS%20%26%20Real%20data/odom_ws) directory some additional information is provided.

## Lidar odometry
Now we are familiar with carla and install it. Here we want to develop a code that tell us the position of our car in each moment and also we can only use lidar sensor. In first phase of our project I am familiar with some point cloud registration algorithms like ICP and NDT. also we know that using lidar alone may have a lot of errors after a long distance. 
you can see the code in [lidar odometry directory](https://github.com/Mahdi-Rahmani/Localization/tree/main/lidar%20odometry) and also some explanations are prepared in its README file.

First we try to do lidar odometry in offline mode and then we should execute our algorithm in real-time mode.

## Use GNSS and IMU with LiDAR (Sensor fusion)
There are various methods and algorithms for sensor fusion. In this research, we used the Extended-Kalman-filter for the sensor fusion, and with this, we tried to increase both safety and redundancy and improve accuracy.

For this purpose, we first tried to Localizing our vehicle with GNSS alone and also with IMU alone and also to observe the EKF output on both of them. This was done in the simulator, and its code can be seen in the [GPS_IMU](https://github.com/Mahdi-Rahmani/Localization/tree/main/GPS_IMU) directory.
After that, we applied EKF on all three sensors together (lidar, IMU and GNSS) and observed its output in realtime in Carla simulator. The code of this section is also located in the [EKF](https://github.com/Mahdi-Rahmani/Localization/tree/main/EKF) directory.

## Experimental part
In this section, our goal is to be able to localizing wheeler body with the help of real data from the lidar sensor. According to the lidar sensor we have and the nodes we create in ROS, this will be possible.

At this stage, we were able to do the localization in the campus of Amir Kabir University of Technology (Tehran Polytechnic). We also managed to do simultaneous location and mapping based on ICP (ICP-based SLAM) and for this we took help from Octomap.
The code of this section is available in [ROS & Real data](https://github.com/Mahdi-Rahmani/Localization/tree/main/ROS%20%26%20Real%20data/odom_ws) directory.

## Final report and presentation file
Finally, all the results and full explanation are given in the report which is available in [Final Report](https://github.com/Mahdi-Rahmani/Localization/tree/main/Final%20Report).
Also, a PowerPoint has been prepared for the final presentation, which can be downloaded and viewed from this [link](https://docs.google.com/presentation/d/1YdPC-5hc70Ywn5VrLdNEWam-g1_IX9SZ/edit?usp=sharing&ouid=112561970420312111928&rtpof=true&sd=true).