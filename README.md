# Lidar based self localization in autonomous vehicles
This is my bachelor's project. I will explain my work step by step here. In this project my purpose is to create a robot that can be self localized in my college environment. For doing this job, after I getting to know some basics of localization I should develop a code for implementing some localization algorithm and also handling some sensor fusion if it is needed. Before testing my code on a real agent in a real environment I should examin it in a simulator, so I choose Carla simulator for this purpose. 

## Proposal
First, after a little research, I had to choose the subject of my project and prepare the proposal file. To check the proposal file in Farsi, you can go [here](https://github.com/Mahdi-Rahmani/Localization/tree/main/proposal) 
According to the time table in my proposal I should do this works in sequence: 

1- Finding related resources and articles and study of them 

2- Getting familiar with carla simulator 

3- Localization with Lidar sensor in carla 

4- Increasing the accuracy of localization that we reached in the previous step 

5- Creating a robot and test my algorithm in college environment 

6- Preparing my final report  
 
First phase, finding related resourses, is done and you can see some related articles in [articles directory](https://github.com/Mahdi-Rahmani/Localization/tree/main/articles) and some good books in [books directory](https://github.com/Mahdi-Rahmani/Localization/tree/main/books). For second phase, I explain completely in next section. 

## Carla simulator 

## Localization with Lidar 
Now we are familiar with carla and install it. Here we want to develop a code that tell us the position of our car in each moment and also we can only use lidar sensor. In first phase of our project I am familiar with some point cloud registration algorithms like ICP and NDT. also we know that using lidar alone may have a lot of errors after a long distance. 
you can see the code in [lidar odometry directory]() and also some explanations are prepared in its README file.
