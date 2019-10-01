#ifndef BASE_WHEEL_DRIVER
#define BASE_WHEEL_DRIVER

#include "robot_config.h"

namespace robot_base
{

class BaseWheelDriver
{

protected:
    BaseWheelDriver() {};

public:
    virtual ~BaseWheelDriver() {};
    virtual void initialize(const std::string& name, const RobotConfig& config) {};

    // set the speed of the motor in radians/second
    virtual void setWheelSpeed(unsigned int wheel_number, float speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) = 0;

    // se the position of the motor in degrees.
    virtual void setWheelPosition(unsigned int wheel_number, float position, PositionUnit position_unit = RADIAN) = 0;

    virtual bool getWheelSpeed(unsigned int wheel_number, float &speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) = 0;

    virtual bool getWheelPosition(unsigned int wheel_number, float &position, PositionUnit position_unit = RADIAN) = 0;
};

}; // namespace base_driver


#endif