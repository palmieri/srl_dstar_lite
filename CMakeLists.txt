cmake_minimum_required(VERSION 2.8.3)
project(srl_dstar_lite)

ADD_DEFINITIONS(-std=c++11) # C++ 11 is required

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  costmap_2d
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

find_package(cmake_modules REQUIRED)

FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

catkin_package(

  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    costmap_2d
    pluginlib
  LIBRARIES
    srl_dstar_lite
)

###########
## Build ##
###########
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
    src/srl_dstar_lite.cpp
    include/srl_dstar_lite/costmap_model.cpp
    src/dstar.cpp
    include/srl_dstar_lite/pathSplineSmoother/pathSplineSmoother.cpp
)

# Plugin
add_library(srl_dstar_lite  ${SOURCES_RRT} ${SOURCES})

target_link_libraries(srl_dstar_lite ${catkin_LIBRARIES} ${BOOST_LIBRARIES} )

add_dependencies(srl_dstar_lite srl_dstar_lite_gencfg ${catkin_EXPORTED_TARGETS})

add_dependencies(srl_dstar_lite nav_msgs_gencpp)


install(TARGETS srl_dstar_lite
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

       )

install(FILES srl_dstar_lite_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
