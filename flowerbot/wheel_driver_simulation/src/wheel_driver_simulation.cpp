#include <kangaroo_driver/kangaroo_driver.h>
#include <cmath>
#include <ros/ros.h>
#include <Kangaroo.h>
#include <Stream.h>
namespace kangaroo_driver
{

WheelDriverSimulation::~WheelDriverSimulation()
{
    
}

void WheelDriverSimulation::initialize(const std::string& name, const RobotConfig& config)
{
    ros::NodeHandle private_nh(name);
    std::map<std::string, int> remaps;
    std::string controller_port;
    private_nh.param<std::string>("controller_port", controller_port, "/dev/ttyUSB0");
    // if (private_nh.getParam("wheel_remap", remaps) == false)
    // {
    //     ROS_ERROR_STREAM("wheel_remap doesn't exist");
    //     exit(1);
    // }

    if (private_nh.getParam("wheel_radius", _wheel_radius) == false)
    {
        ROS_ERROR("no wheel radius is assigned.");
        //exit(1);
    }
    else if (_wheel_radius <= 0)
    {
        ROS_ERROR("wheel radius must be positive.");
    }

    // config.initMotorRemaps(_wheel_remap);
    // for (auto const &element : _wheel_remap)
    // {
    //     if (_wheel_remap.find(element.first) == _wheel_remap.end())
    //     {
    //         ROS_ERROR_STREAM("Cound not find a matched wheel named: " << element.first << "in the robot configuration");
    //         exit(1);
    //     }
    //     else
    //     {
    //         if (element.second<0)
    //         {
    //             ROS_ERROR_STREAM("Motor number for wheel " << element.first << " cannot be negative.");
    //             exit(1);
    //         }
    //         else
    //             _wheel_remap[element.first] = element.second;
    //     }
    // }

    
    if (_serial_port.begin(controller_port) == false)
    {
        ROS_ERROR_STREAM("Can't open the controller port: " << controller_port);
        exit(1);
    }

    KangarooSerial K(_serial_port);
    _wheels.push_back(KangarooChannel(K, '1', 128));
    _wheels.push_back(KangarooChannel(K, '2', 128));
    _wheels.push_back(KangarooChannel(K, '1', 129));
    _wheels.push_back(KangarooChannel(K, '2', 129));
    for (int i = 0; i < _wheels.size(); i++)
    {
        KangarooError e= _wheels[i].start();
        if (e!=KANGAROO_NO_ERROR)
        {
            ROS_ERROR_STREAM("can't start wheel " << i);
            exit(0);
        }
        _wheels[i].units(1, 12);
    }
}

// set the speed of the wheel in radians/second
void WheelDriverSimulation::setWheelSpeed(unsigned int wheel_number, float speed, SpeedUnit speed_unit)
{
    float s = speed;
    switch (speed_unit)
    {
    case RADIAN_PER_SECOND:
        s = speed * M_1_PIf32 * 180;
        break;
    case ROTATION_PER_MINITE:
        s = speed * 60;
        break;
    case DEGREE_PER_SECOND:
        break;
    case METER_PER_SECOND:
        s = speed / _wheel_radius * M_1_PIf32 * 180;
    default:
        break;
    }
    _wheels[wheel_number].s((int32_t)s);
}

// se the position of the wheel in degrees.
void WheelDriverSimulation::setWheelPosition(unsigned int wheel_number, float position, PositionUnit position_unit)
{
    float p = position;
    switch (position_unit)
    {
    case RADIAN:
        p = position * M_1_PIf32 * 180;
        break;
    case DEGREE:
        break;
    default:
        break;
    }
    _wheels[wheel_number].p((int32_t)p);
}

bool WheelDriverSimulation::getWheelSpeed(unsigned int wheel_number, float &speed, SpeedUnit speed_unit)
{
    KangarooStatus status = _wheels[wheel_number].getS();
    speed = status.value();
    if (status.error() == KANGAROO_NO_ERROR)
    {
        switch (speed_unit)
        {
        case RADIAN_PER_SECOND:
            speed = speed * M_PI / 180;
            break;
        case ROTATION_PER_MINITE:
            speed = speed / 60;
            break;
        case DEGREE_PER_SECOND:
            break;
        case METER_PER_SECOND:
            speed = speed * _wheel_radius * M_PI / 180;
        default:
            break;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool WheelDriverSimulation::getWheelPosition(unsigned int wheel_number, float &position, PositionUnit position_unit)
{
    KangarooStatus status = _wheels[wheel_number].getP();
    position = status.value();
    if (status.error() == KANGAROO_NO_ERROR)
    {
        switch (position_unit)
        {
        case RADIAN:
            position = position * M_PI / 180;
            break;
        case DEGREE:
            break;
        default:
            break;
        }
        return true;
    }
    else
    {
        return false;
    }
}

} // namespace kangaroo_controller_driver

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kangaroo_driver::WheelDriverSimulation, robot_base::BaseWheelDriver);