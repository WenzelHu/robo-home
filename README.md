# Compliant Grasping

1.Introduction
----------
One of the most convenient and advanced ways of moving robot’s arm to a desired position and grasp an object and avoid collision with obstacles is using *moveit* motion planning framework. Once the desired goal is sent to the *move_it* node, it will calculate a path that pypasses obstacles and reaches the goal. And by execuing this path, *move_it* will communicate with the physical interface of robot and the robot will be moved.

However, in our ﬁnal project, we propose to manipulate the robot without using *move_it*. According to some control laws that will implement impedance and admittance control of arm andgripper, some values will be calculated and directly sent to the physical interface of robot. Moreover, skins are used to modify the values sent to robot. By incorporating controllers and skin, complinat grapsing adaptve to the environment is implemented. General structure of the project is as following
![figure1](https://github.com/WenzelHu/robo-home/raw/master/imgs/figure1.png)
