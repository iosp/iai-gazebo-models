cmake_minimum_required(VERSION 2.8.3)
project(velodyne)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  tf
  message_generation
  genmsg
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(tf REQUIRED)
find_package(gazebo REQUIRED)



## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )



catkin_package(
  DEPENDS 
    roscpp 
    tf
    gazebo_ros 
    std_msgs
   CATKIN_DEPENDS message_runtime
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(velodyne16 src/velodyne16.cpp)
target_link_libraries(velodyne16 ${GAZEBO_libraries} ${catkin_LIBRARIES} )

add_library(vlp src/VLPCommunication.cpp)
target_link_libraries(vlp ${GAZEBO_libraries} ${catkin_LIBRARIES} )







