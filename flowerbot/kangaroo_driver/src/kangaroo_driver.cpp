#include <kangaroo_driver/kangaroo_driver.h>
#include <cmath>
#include <ros/ros.h>
#include <Kangaroo.h>
#include <Stream.h>
namespace kangaroo_driver
{

KangarooDriver::~KangarooDriver()
{
    for (int i = 0; i < _wheels.size(); i++)
    {
        if (_wheels[i].driving_wheel_ptr != nullptr)
        {
            KangarooError e = _wheels[i].driving_wheel_ptr->powerDownAll();
        }
    }
    _serial_port.end();

}

void KangarooDriver::initialize(const std::string& name, const RobotConfig& config)
{
    ROS_INFO_STREAM("namespace is " << name);
    ros::NodeHandle private_nh(name);
    std::map<std::string, int> remaps;
    std::string controller_port;
    private_nh.param<std::string>("controller_port", controller_port, "/dev/ttyUSB0");
    private_nh.param<int>("max_steering_speed", _max_steering_speed, 300);
    private_nh.param<int>("driving_speed_ramp", _driving_speed_ramp, 200);

    _robot_config=config;

    if (private_nh.getParam("motor_calibration_zero", _motor_calibration_zero) == false)
    {
        ROS_INFO("No motor_calibration_zero is found. Using default value {0, 0, 0, 0}");
        _motor_calibration_zero={0, 442, 0, 828};
    }

    if (private_nh.getParam("motor_calibration_scale", _motor_calibration_scale) == false)
    {
        ROS_INFO("No motor_calibration_scale is found. Using default value {1, 1, 1, 1}");
        _motor_calibration_scale={-33.766, 202.6, -33.766, 202.6};
    }


    // if (private_nh.getParam("wheel_remap", remaps) == false)
    // {
    //     ROS_ERROR_STREAM("wheel_remap doesn't exist");
    //     exit(1);
    // }

    // if (private_nh.getParam("wheel_radius", _wheel_radius) == false)
    // {
    //     ROS_ERROR("no wheel radius is assigned.");
    //     //exit(1);
    // }
    // else if (_wheel_radius <= 0)
    // {
    //     ROS_ERROR("wheel radius must be positive.");
    // }

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

    if (_serial_port.begin(controller_port, 100, B115200) == false)
    {
        ROS_ERROR_STREAM("Can't open the controller port: " << controller_port);
        exit(1);
    }
    _kangaroo_serial_ptr.reset(new KangarooSerial(_serial_port));
    KangarooWheel k_w;
    k_w.driving_wheel_ptr.reset(new KangarooChannel(*_kangaroo_serial_ptr, '1', 129));
    //k_w.driving_wheel_ptr->commandRetryInterval(200);
    k_w.driving_wheel_zero = _motor_calibration_zero[0];
    k_w.driving_wheel_scale = _motor_calibration_scale[0];
    k_w.steering_wheel_ptr.reset(new KangarooChannel(*_kangaroo_serial_ptr, '2', 129));
    k_w.steering_wheel_zero = _motor_calibration_zero[1];
    k_w.steering_wheel_scale = _motor_calibration_scale[1];
    _wheels.push_back(k_w);

    k_w.driving_wheel_ptr.reset(new KangarooChannel(*_kangaroo_serial_ptr, '1', 128));
    k_w.driving_wheel_zero = _motor_calibration_zero[2];
    k_w.driving_wheel_scale = _motor_calibration_scale[2];
    k_w.steering_wheel_ptr.reset(new KangarooChannel(*_kangaroo_serial_ptr, '2', 128));
    k_w.steering_wheel_zero = _motor_calibration_zero[3];
    k_w.steering_wheel_scale = _motor_calibration_scale[3];
    _wheels.push_back(k_w);

    // _wheels.push_back(KangarooChannel(K, '2', 128));
    // _wheels.push_back(KangarooChannel(K, '1', 129));
    // _wheels.push_back(KangarooChannel(K, '2', 129));
    for (int i = 0; i < _wheels.size(); i++)
    {
        if (_wheels[i].driving_wheel_ptr != nullptr)
        {
            KangarooError e = _wheels[i].driving_wheel_ptr->start();
            if (e!=KANGAROO_NO_ERROR)
            {
                ROS_FATAL_STREAM("can't start driving wheel " << i);
                exit(0);
            }
            // _wheels[i].driving_wheel_ptr->units(1, 12);
            
        }
        if (_wheels[i].steering_wheel_ptr != nullptr)
        {
            KangarooError e= _wheels[i].steering_wheel_ptr->start();
            if (e!=KANGAROO_NO_ERROR && e!=KANGAROO_NOT_HOMED)
            {
                ROS_FATAL_STREAM("can't start steering wheel " << i);
                //exit(0);
            }
            // _wheels[i].steering_wheel_ptr->units(5, 266);
            if (e ==KANGAROO_NOT_HOMED)
            {
                KangarooMonitor motor_monitors=_wheels[i].steering_wheel_ptr->home().wait();
                if (motor_monitors.status().error() != KANGAROO_NO_ERROR)
                {
                    ROS_FATAL_STREAM("Couldn't home steering wheel " << i);
                    exit(0);
                }
            }
            ROS_INFO_STREAM("reposition steering wheel "<< i <<" to 0 degree. ");
            _wheels[i].steering_wheel_ptr->p(_wheels[i].steering_wheel_zero, _max_steering_speed).wait();

            //int current_position=_wheels.at(i).steering_wheel_ptr->getP().value();
            //current_position=1;
        }
    }
}

// set the speed of the wheel in radians/second
void KangarooDriver::setWheelSpeed(unsigned int wheel_number, float speed, SpeedUnit speed_unit)
{
    float s = speed;
    switch (speed_unit)
    {
    case RADIAN_PER_SECOND:
        break;
    case ROTATION_PER_MINITE:
        s = speed * M_PI / 30;
        break;
    case DEGREE_PER_SECOND:
        s = speed/180 * M_PI;
        break;
    case METER_PER_SECOND:
        s = speed / _robot_config.getWheelRadius(wheel_number);
    default:
        break;
    }
    _wheels.at(wheel_number).driving_wheel_ptr->s((int32_t)(s*_wheels.at(wheel_number).driving_wheel_scale + _wheels.at(wheel_number).driving_wheel_zero), _driving_speed_ramp);
}

// set the position of the wheel in radiance.
void KangarooDriver::setWheelPosition(unsigned int wheel_number, float position, PositionUnit position_unit)
{
    float p = position;
    switch (position_unit)
    {
    case RADIAN:
        break;
    case DEGREE:
        p = position * M_PI / 180;
        break;
    default:
        break;
    }
	//ROS_INFO_STREAM("Position raw: "<< (int32_t)(p*_wheels.at(wheel_number).steering_wheel_scale+_wheels.at(wheel_number).steering_wheel_zero));
    _wheels.at(wheel_number).steering_wheel_ptr->p((int32_t)(p*_wheels.at(wheel_number).steering_wheel_scale+_wheels.at(wheel_number).steering_wheel_zero), _max_steering_speed);
}

bool KangarooDriver::getWheelSpeed(unsigned int wheel_number, float &speed, SpeedUnit speed_unit)
{
    KangarooStatus status = _wheels.at(wheel_number).driving_wheel_ptr->getS();
    speed = (status.value() - _wheels.at(wheel_number).driving_wheel_zero)/_wheels.at(wheel_number).driving_wheel_scale;
    if (status.error() == KANGAROO_NO_ERROR)
    {
        switch (speed_unit)
        {
        case RADIAN_PER_SECOND:
            break;
        case ROTATION_PER_MINITE:
            speed = speed * M_1_PI * 30;
            break;
        case DEGREE_PER_SECOND:
            speed = speed * M_1_PI * 180;
            break;
        case METER_PER_SECOND:
            speed = speed * _robot_config.getWheelRadius(wheel_number);
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

bool KangarooDriver::getWheelPosition(unsigned int wheel_number, float &position, PositionUnit position_unit)
{
    KangarooStatus status = _wheels.at(wheel_number).steering_wheel_ptr->getP();
    position = (status.value() - _wheels.at(wheel_number).steering_wheel_zero)/_wheels.at(wheel_number).steering_wheel_scale;
    if (status.error() == KANGAROO_NO_ERROR)
    {
        switch (position_unit)
        {
        case RADIAN:
            break;
        case DEGREE:
            position = position * 180 * M_1_PI;
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
PLUGINLIB_EXPORT_CLASS(kangaroo_driver::KangarooDriver, robot_base::BaseWheelDriver);
