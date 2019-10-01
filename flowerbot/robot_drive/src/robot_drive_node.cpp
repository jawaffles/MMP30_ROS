#include <robot_drive/robot_drive.h>
#include <ros/ros.h>
int main(int argc, char** argv) {
  std::cout << "start.";
  ros::init(argc, argv, "robot_drive");
  ros::NodeHandle pnh("~");
  try {
    robot_drive::RobotDriverRos single_node(pnh, "");
    single_node.spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
  return 0;
}