cmake_minimum_required(VERSION 2.8.3)
project(cartesian_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  kdl_parser 
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  visualization_msgs
)


###############
#Eigen
###############
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp 
)


###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  #${EIGEN3_INCLUDE_DIR}
	
)

add_library(cartesian_controller_class src/cartesian_controller_class.cpp)
#target_link_libraries(turtle_ros_tutorial ${roscpp_LIBRARIES} ${catkin_LIBRARIES})

add_executable(cartesian_controller src/cartesian_controller.cpp)
target_link_libraries(cartesian_controller
   ${catkin_LIBRARIES} cartesian_controller_class
)

#add_dependencies(cartesian_controller ${catkin_EXPORTED_TARGETS})
