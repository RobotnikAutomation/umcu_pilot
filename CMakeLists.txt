cmake_minimum_required(VERSION 2.8.3)
project(sermas_pilot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  odin_msgs
  rcomponent
  robot_simple_command_manager_msgs
  robotnik_msgs
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    move_base_msgs  
    odin_msgs
    rcomponent
    robot_simple_command_manager_msgs
    robotnik_msgs
    roscpp
    std_msgs
    std_srvs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/sermas_pilot.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(sermas_pilot_node src/sermas_pilot_node.cpp)
add_dependencies(sermas_pilot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a executable target against
target_link_libraries(sermas_pilot_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} sermas_pilot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
