# Compliant Grasping

This is the project for object detection and stiffness control of robot wrist and grippers.

For detection of objects and estimation of objects' position, [YOLO](https://pjreddie.com/darknet/yolo/) is utilized to detect objects in a 2D images and generated the pixel coordinates for the four tips of one bounding box that surrounds a certain detected object. Along with a 2D image, there is a point cloud sent from the kinect camera on robot. Point cloud consists of depth information for every pixel in the 2D image. Via point cloud, we can get the 3D position of a certain object. This position
information will be used in the subsequent grasping of this object.

For cartesian control, we calculate forward and inverse kinematics for robot's arms. In this way, we are able to transform the angular positions and velocities of seven joints on the arm into a cartesian coordinate of the end effector with respect to robot's root joint and vice versa.

For stiffness control of robot wrist and grippers, goal position for robot's end effector to reach will be modified based on current external force applied on it. Thus, if robot's gripper or wrist confront an obstacle, in this case, robot's end effector will sense a strong force or torque, and goal position will be changed. Robot will not force to get to its oringal goal which may cause damges to robot. Instead, it will stop near the obstacle since its modified goal position is here depending on currently sensed external force. Stiffness control enable robots to adapt to the changed or not accurately sensed environment.

