#ifndef BASIC_ROBOT_CONTROLLER
#define BASIC_ROBOT_CONTROLLER
#include <robot_base/base_robot_controller.h>
#include <robot_base/base_wheel_driver.h>
#include <robot_base/robot_config.h>

namespace robot_base
{

class BasicRobotController : public BaseRobotController
{
public:
    BasicRobotController() {}
    void initialize(const std::string& name) override;
    bool calculateWheelCommand(std::vector<WheelCommand>& wheel_command, const geometry_msgs::TwistConstPtr& cmd_vel, const RobotConfig& config) override;
    bool calculateOdometry(nav_msgs::Odometry &odom, std::vector<WheelCommand>& wheel_command, const RobotConfig& config) override;
};
}; // namespace robot_base


#endif