cmake_minimum_required(VERSION 2.8.3)
project(nomad)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin  REQUIRED
        COMPONENTS
        roscpp
        rospy
        geometry_msgs
        )

## System dependencies are found with CMake's conventions
INCLUDE(FindPkgConfig)
find_package(SDL2 REQUIRED)
PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nomad
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        ${SDL2IMAGE_INCLUDE_DIRS}
)

add_executable(path_planner_node nodes/visualizer.cpp)

target_link_libraries(path_planner_node ${catkin_LIBRARIES} ${SDL2_LIBRARY} ${SDL2IMAGE_LIBRARIES})

install(PROGRAMS
   nodes/point2Actions.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark executables and/or libraries for installation
 install(TARGETS path_planner_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
