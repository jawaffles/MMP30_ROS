cmake_minimum_required(VERSION 2.8.3)
project(robot_drive)
add_compile_options(-std=c++11)
# # Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    pluginlib
    robot_base
)
# catkin_python_setup()


# Declare that this catkin package's runtime dependencies
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    std_msgs
    robot_base
    geometry_msgs
    nav_msgs
    pluginlib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/robot_drive.cpp
    src/robot_drive_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
  )

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
