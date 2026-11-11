cmake_minimum_required(VERSION 3.14)
project(sonar_context_slam)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge
             image_transport tf2_ros)
find_package(GTSAM REQUIRED)
# ROS Noetic's OpenCV 4 (cv_bridge's); DISO's image also has an OpenCV 3.4 in /usr/local
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
catkin_package()

# the Sonar Context library and the ROS-free SLAM back-end of this repository
set(SONAR_CONTEXT_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../.. CACHE PATH "sonar_context repository")
set(SONAR_CONTEXT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
add_subdirectory(${SONAR_CONTEXT_ROOT}/cpp ${CMAKE_BINARY_DIR}/sonar_context_core)
add_library(sonar_context_slam_core ${SONAR_CONTEXT_ROOT}/applications/slam/core/sonar_slam.cpp
            ${SONAR_CONTEXT_ROOT}/applications/slam/core/dense_registration.cpp)
target_include_directories(sonar_context_slam_core PUBLIC ${SONAR_CONTEXT_ROOT}/applications/slam/core)
target_link_libraries(sonar_context_slam_core PUBLIC sonar_context::sonar_context gtsam ${OpenCV_LIBS})
target_include_directories(sonar_context_slam_core PUBLIC ${OpenCV_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(sonar_context_slam_node src/slam_node.cpp)
target_link_libraries(sonar_context_slam_node sonar_context_slam_core ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS sonar_context_slam_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# multi-session alignment (ROS-free)
add_executable(sonar_context_align_sessions ${SONAR_CONTEXT_ROOT}/applications/multi-session-slam/align_sessions.cpp)
target_link_libraries(sonar_context_align_sessions sonar_context_slam_core)
install(TARGETS sonar_context_align_sessions RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# offline replay (ROS-free)
add_executable(sonar_context_replay ${SONAR_CONTEXT_ROOT}/applications/slam/tools/replay.cpp)
target_link_libraries(sonar_context_replay sonar_context_slam_core)
install(TARGETS sonar_context_replay RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# test of the dense registration (simulated views)
add_executable(sonar_context_test_dense ${SONAR_CONTEXT_ROOT}/applications/slam/tests/test_dense.cpp)
target_link_libraries(sonar_context_test_dense sonar_context_slam_core)
