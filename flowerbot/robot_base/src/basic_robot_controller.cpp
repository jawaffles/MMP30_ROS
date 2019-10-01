#include <basic_robot_controller/basic_robot_controller.h>
#include <ros/ros.h>
namespace robot_base
{

    void BasicRobotController::initialize(const std::string& name)
    {

    }
bool BasicRobotController::calculateWheelCommand(std::vector<WheelCommand> &wheel_command, const geometry_msgs::TwistConstPtr &cmd_vel, const RobotConfig &config)
{
    double ICR_x, ICR_y;
    double s;
    double beta;
    WheelCommand w_c; 
	//ROS_INFO_STREAM("cmd_vel: x=" << cmd_vel->linear.x <<", y="<<cmd_vel->linear.y << ", angle_z=" << cmd_vel->angular.z);
    for (int i = 0; i < config.getWheelCount(); i++)
    {
        if (cmd_vel->angular.z <= 1e-6 && cmd_vel->angular.z >= -1e-6)
        {
            ICR_x = 0;
            ICR_y = 0;
            s = sqrt(cmd_vel->linear.x * cmd_vel->linear.x + cmd_vel->linear.y * cmd_vel->linear.y);
            beta = atan2(cmd_vel->linear.y, cmd_vel->linear.x);
        }
        else
        {
            ICR_x = -cmd_vel->linear.y / cmd_vel->angular.z + config.getComX();
            ICR_y = cmd_vel->linear.x / cmd_vel->angular.z + config.getComY();
			//ROS_INFO_STREAM("ICR_x "<<ICR_x<<"ICR_y"<<ICR_y);
            beta = atan2(config.getWheelPositionX(i) - ICR_x, ICR_y - config.getWheelPositionY(i));
            s = cmd_vel->angular.z * sqrt((config.getWheelPositionX(i) - ICR_x) * (config.getWheelPositionX(i) - ICR_x) + (ICR_y - config.getWheelPositionY(i)) * (ICR_y - config.getWheelPositionY(i)));
        }
        s=s/config.getWheelRadius(i);
        //ROS_INFO_STREAM("wheel "<<i<<": angle="<<beta/M_PI*180<<"deg, speed="<<s<<" rad/s");

        if (beta>M_PI_2)
        {
            beta-=M_PI;
            s=-s;
        }

        if (beta<-M_PI_2)
        {
            beta+=M_PI;
            s=-s;
        }

        w_c.wheel_id=config.getWheelId(i);
        w_c.wheel_rotation=beta;
        w_c.wheel_speed=s;
        wheel_command.push_back(w_c);
        
    }
}
bool BasicRobotController::calculateOdometry(nav_msgs::Odometry &odom, std::vector<WheelCommand> &wheel_command, const RobotConfig &config)
{

}
}; // namespace robot_base

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_base::BasicRobotController, robot_base::BaseRobotController);
