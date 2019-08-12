cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation)
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  perception_msgs
  darknet_ros_msgs
)


#add_message_files(FILES ObjectDetection.msg)
#add_service_files(FILES DetectObject.srv)
#generate_messages(DEPENDENCIES std_msgs)



catkin_package()
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}  
)


## Declare a cpp executable
add_executable(plane_segmentation src/PlaneSegmentation.cpp)
add_dependencies(plane_segmentation perception_msgs_generate_messages_cpp)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(
  ${PCL_INCLUDE_DIRS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(plane_segmentation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})




