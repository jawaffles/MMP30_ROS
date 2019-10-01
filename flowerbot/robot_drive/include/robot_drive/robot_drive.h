#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <robot_base/base_wheel_driver.h>
#include <robot_base/base_robot_controller.h>
#include <robot_base/robot_config.h>
#include <pluginlib/class_loader.hpp>


namespace robot_drive
{
class RobotDriverRos
{
public:
    RobotDriverRos(const ros::NodeHandle& pnh, const std::string& ns = std::string());
    ~RobotDriverRos();
    void cmdVelCallback(const geometry_msgs::TwistConstPtr &cmd_ve);
    void publishOdometry();
    void spin();


private:
    ros::NodeHandle pnh_;
    ros::Publisher _odometry_pub;
    ros::Publisher _joint_state_publisher;
    ros::Subscriber _cmd_vel_sub;
    std::string _robot_config_file;
    nav_msgs::Odometry _odom;
    sensor_msgs::JointState _joint_states;


    int _motor_num;
    bool _simulation;
    std::vector<robot_base::WheelCommand> _wheel_command;
    boost::shared_ptr<robot_base::BaseWheelDriver> _wheel_driver;
    pluginlib::ClassLoader<robot_base::BaseWheelDriver> _wheel_driver_loader;

    boost::shared_ptr<robot_base::BaseRobotController> _robot_controller;
    pluginlib::ClassLoader<robot_base::BaseRobotController> _robot_controller_loader;

    robot_base::RobotConfig _robot_config;
    std::shared_ptr<ros::Rate> _rate_ptr;

};

};
