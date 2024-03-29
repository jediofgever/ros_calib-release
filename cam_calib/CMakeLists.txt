cmake_minimum_required(VERSION 2.8.3)
project(cam_calib)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    moveit_core
    moveit_ros_planning
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_ros
    geometry_msgs
    visp_camera_calibration
    visp_bridge
    common
)
find_package(Qt5Widgets REQUIRED)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  ${THIS_PACKAGE_INCLUDE_DIRS} 
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  common
)

include_directories( SYSTEM include ${THIS_PACKAGE_INCLUDE_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} ${common_INCLUDE_DIRS})
include_directories( ${visp_camera_calibration_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########
add_executable(${PROJECT_NAME}
src/CamCalibration.cpp 
src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#includes
install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)
#Install Directory
install(DIRECTORY cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})