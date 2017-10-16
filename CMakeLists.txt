cmake_minimum_required(VERSION 2.8.3)
project(astrobee_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

include_directories(
   include
   src
   ${OCTOMAP_INCLUDE_DIRS}
   )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  octomap
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
)

find_package(cmake_modules REQUIRED
             Eigen REQUIRED)

link_directories(
    ${PCL_LIBRARY_DIRS})

add_service_files( FILES
  updateMapFloatParam.srv
  updateMapBoolParam.srv
  updateMapInflation.srv
  voidService.srv
)

generate_messages( DEPENDENCIES
  std_msgs
)
    

###################################
## catkin specific configuration ##
###################################
catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(astrobee_mapper
  src/astrobee_mapper.cpp
  # src/polynomials.cpp
  # src/segment_functions.cpp
  # src/sampledTrajectory.cpp
  src/octoClass.cpp
  src/tf_class.cpp
  src/HelperFcns/visualizationFunctions.cpp
  src/HelperFcns/QuatRotEuler.cpp
  src/structs.cpp
  src/Services/services.cpp
  src/threads/threads.cpp
  src/Callbacks/callbacks.cpp
)

target_link_libraries(astrobee_mapper
${catkin_LIBRARIES}
${OCTOMAP_LIBRARIES}
)

#This makes sure that messages and services are compiled before the rest
add_dependencies(astrobee_mapper astrobee_mapper_generate_messages_cpp)
add_dependencies(astrobee_mapper ${catkin_EXPORTED_TARGETS})