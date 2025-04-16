cmake_minimum_required(VERSION 3.0.2)
project(itri_imu_usb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  tf2
  Boost
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs serial tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(itri_imu_usb src/itri_imu_usb.cpp)
target_link_libraries(itri_imu_usb ${catkin_LIBRARIES})

add_executable(itri_imu_usb_sss src/itri_imu_usb_sss.cpp)
target_link_libraries(itri_imu_usb_sss ${catkin_LIBRARIES})

add_executable(sensor_filter_sss src/sensor_filter_sss.cpp)
target_link_libraries(sensor_filter_sss ${catkin_LIBRARIES})

add_executable(moving_average_filter src/moving_average_filter.cpp)
target_link_libraries(moving_average_filter ${catkin_LIBRARIES})

add_executable(low_pass_filter src/low_pass_filter.cpp)
target_link_libraries(low_pass_filter ${catkin_LIBRARIES})

add_executable(butterworth_filter src/butterworth_filter.cpp)
target_link_libraries(butterworth_filter ${catkin_LIBRARIES})

add_executable(weighted_moving_average_filter src/weighted_moving_average_filter.cpp)
target_link_libraries(weighted_moving_average_filter ${catkin_LIBRARIES})