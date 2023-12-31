cmake_minimum_required(VERSION 3.0.2)
project(trajectory_library)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  tf2_msgs
  visualization_msgs
  mmpug_msgs
)

find_package(  Eigen3 REQUIRED )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# find_package(CUDA 11.3 REQUIRED)
# find_package(OpenMP)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES traj_lib
    CATKIN_DEPENDS 
        roscpp 
        std_msgs 
        geometry_msgs 
        nav_msgs
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(traj_lib
    src/Costmap.cpp
    src/TrajectoryLibraryManager.cpp
    src/Trajectory.cpp
    src/CostFunction.cpp
    src/GoalManager.cpp
)
target_link_libraries(traj_lib ${catkin_LIBRARIES})
add_dependencies(traj_lib ${catkin_EXPORTED_TARGETS})

add_executable(TrajectoryLibraryWrapper src/TrajectoryLibraryWrapper.cpp)
target_link_libraries(TrajectoryLibraryWrapper
    traj_lib
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)
add_dependencies(TrajectoryLibraryWrapper
    ${catkin_EXPORTED_TARGETS})
    

## Testing
catkin_install_python(PROGRAMS mover.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(my_turtlebot_sim.test)
endif()
