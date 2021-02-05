cmake_minimum_required(VERSION 2.8.3)
project(flir_ptu_ethernet)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  rcomponent
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS
    #include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    rcomponent
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#  src/flir_ptu_ethernet.cpp
#)

# add_executable(flir_ptu_ethernet_node src/flir_ptu_ethernet_node.cpp)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(flir_ptu_ethernet_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(flir_ptu_ethernet_node
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/flir_ptu_ethernet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} flir_ptu_ethernet_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
