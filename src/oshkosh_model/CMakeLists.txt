cmake_minimum_required(VERSION 2.8.3)
project(oshkosh_model)

SET(CMAKE_CXX_FLAGS "-std=c++0x")


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  geometry_msgs
  dynamic_reconfigure
)



find_package(Boost REQUIRED COMPONENTS 
   thread 
   system 
)

find_package(gazebo REQUIRED)

generate_dynamic_reconfigure_options(
       cfg/oshkosh_wheels_shaft_dumping.cfg     
       cfg/oshkosh_wheels_drive.cfg
       cfg/oshkosh_wheels_steer.cfg
)   



link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  std_msgs
  #dynamic_reconfigure
)



include_directories(  ${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(oshkosh_wheel_shaft_suspension_plugin SHARED src/oshkosh_wheel_shaft_suspension_plugin.cc)
target_link_libraries(oshkosh_wheel_shaft_suspension_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(oshkosh_wheel_shaft_suspension_plugin ${PROJECT_NAME}_gencfg)


add_library(oshkosh_wheel_drive_plugin SHARED src/oshkosh_wheel_drive_plugin.cc)
target_link_libraries(oshkosh_wheel_drive_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(oshkosh_wheel_drive_plugin ${PROJECT_NAME}_gencfg)


add_library(oshkosh_wheel_steer_plugin SHARED src/oshkosh_wheel_steer_plugin.cc)
target_link_libraries(oshkosh_wheel_steer_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
add_dependencies(oshkosh_wheel_steer_plugin ${PROJECT_NAME}_gencfg)


add_library(oshkosh_TF_pub_plugin SHARED src/oshkosh_TF_pub_plugin.cc)
target_link_libraries(oshkosh_TF_pub_plugin ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
