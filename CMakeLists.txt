cmake_minimum_required(VERSION 2.8.3)
project(yolo_staple_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        ros_yolo
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(STAPLE
        include/fhog.h
        include/sse.hpp
        src/fhog.cpp
        include/staple_tracker.hpp
        src/staple_tracker.cpp)
target_link_libraries(STAPLE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )


add_executable(tracker_yolo
        App/tracker_yolo.cpp
        )
target_link_libraries(tracker_yolo
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        STAPLE
        )