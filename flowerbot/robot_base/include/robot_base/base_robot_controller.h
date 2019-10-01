#ifndef BASE_ROBOT_CONTROLLER
#define BASE_ROBOT_CONTROLLER
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "robot_config.h"

namespace robot_base
{

class BaseRobotController
{
public:
    virtual ~BaseRobotController() {};
    virtual void initialize(const std::string& name) {};
    virtual bool calculateWheelCommand(std::vector<WheelCommand>& wheel_command, const geometry_msgs::TwistConstPtr& cmd_vel, const RobotConfig& config) = 0;
    virtual bool calculateOdometry(nav_msgs::Odometry &odom, std::vector<WheelCommand>& wheel_command, const RobotConfig& config) = 0;
protected:
    BaseRobotController() {};
};


}; // namespace robot_base
#endif