cmake_minimum_required(VERSION 2.8.3)
project(abb_irb120)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

FIND_PACKAGE (RBDL REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/abb_irb120_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

catkin_package(
   #INCLUDE_DIRS include
   LIBRARIES abb_irb120
   DEPENDS gazebo_ros roscpp rospy std_msgs
   #DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
