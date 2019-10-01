cmake_minimum_required(VERSION 2.8.3)
project(robot_base)
add_compile_options(-std=c++11)
# # Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
geometry_msgs
nav_msgs
)
# catkin_python_setup()


# Declare that this catkin package's runtime dependencies
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_test
  src/robot_config.cpp
  src/robot_config_test.cpp
)

target_link_libraries(${PROJECT_NAME}_test 
  ${catkin_LIBRARIES}
  yaml-cpp
)
# add_dependencies(${PROJECT_NAME}_test ${PROJECT_NAME})

add_library(${PROJECT_NAME}
  src/robot_config.cpp
  #src/basic_robot_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_library(basic_robot_controller
  src/robot_config.cpp
  src/basic_robot_controller.cpp
)

target_link_libraries(basic_robot_controller
  ${catkin_LIBRARIES}
  yaml-cpp
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
