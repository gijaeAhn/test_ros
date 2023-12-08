cmake_minimum_required(VERSION 3.16.3)
project(realsense_test)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
add_definitions("-std=c++14")

find_package(OpenCV REQUIRED)

find_package(realsense2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             tf
             sensor_msgs
             cv_bridge)

find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_ros ros_realsense_test.cc )
target_link_libraries(${PROJECT_NAME}_ros ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_executable(pcl_ros pcl_ros.cc)
target_link_libraries(pcl_ros ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME}_ros realsense2::realsense2)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    tf
    sensor_msgs

)
