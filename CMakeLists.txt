cmake_minimum_required(VERSION 2.8.3)
project(mw_mdc24d200d_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mw_mdc24d200d_ctrl src/mw_mdc24d200d_ctrl.cpp)

target_link_libraries(mw_mdc24d200d_ctrl
  ${catkin_LIBRARIES}
)

