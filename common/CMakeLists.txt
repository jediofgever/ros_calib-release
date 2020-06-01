cmake_minimum_required(VERSION 2.8.3)
project(common)

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
)
find_package(Qt5Widgets REQUIRED)


catkin_package(
  LIBRARIES common
  INCLUDE_DIRS
  ${THIS_PACKAGE_INCLUDE_DIRS} include
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
)

include_directories( SYSTEM include ${THIS_PACKAGE_INCLUDE_DIRS})
include_directories( ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########
add_library(${PROJECT_NAME}
src/RobotController.cpp
src/RobotPoseGenerator.cpp 
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} Qt5::Widgets)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
#   # myfile1
#   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)