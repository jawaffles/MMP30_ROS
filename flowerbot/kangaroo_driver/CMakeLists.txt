cmake_minimum_required(VERSION 2.8.3)
project(kangaroo_driver)
add_compile_options(-std=c++11)
# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robot_base
  pluginlib
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
    roscpp
    std_msgs
    robot_base
    pluginlib
  )


include_directories(
    include
    thirdparty/kangaroo_controller/include
    ${catkin_INCLUDE_DIRS}
  )


add_library(kangaroo_lib
  thirdparty/kangaroo_controller/src/KangarooChannel.cpp
  thirdparty/kangaroo_controller/src/KangarooCommandWriter.cpp
  thirdparty/kangaroo_controller/src/KangarooCRC.cpp
  thirdparty/kangaroo_controller/src/KangarooMonitor.cpp
  thirdparty/kangaroo_controller/src/KangarooReplyReader.cpp
  thirdparty/kangaroo_controller/src/KangarooReplyReceiver.cpp
  thirdparty/kangaroo_controller/src/KangarooSerial.cpp
  thirdparty/kangaroo_controller/src/KangarooStatus.cpp
  thirdparty/kangaroo_controller/src/KangarooTimeout.cpp
  thirdparty/kangaroo_controller/src/Stream.cpp
)

add_library(${PROJECT_NAME}
  src/kangaroo_driver.cpp
)

target_link_libraries(${PROJECT_NAME} 
  #${catkin_LIBRARIES}
  kangaroo_lib
)

add_executable(${PROJECT_NAME}_test
  src/kangaroo_driver.cpp
  src/test.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES}
  kangaroo_lib
)
