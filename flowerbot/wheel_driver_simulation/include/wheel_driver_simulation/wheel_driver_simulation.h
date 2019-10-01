#ifndef WHEEL_DRIVER_SIMULATION
#define WHEEL_DRIVER_SIMULATION

#include <robot_base/base_wheel_driver.h>
#include <robot_base/robot_config.h>
#include <map>


namespace base_robot_controller
{
    class RobotConfig;
}

namespace wheel_driver_simulation
{

using namespace robot_base;

class WheelDriverSimulation : public BaseWheelDriver
{
public:
    WheelDriverSimulation() {}
    ~WheelDriverSimulation();
    void initialize(const std::string& name, const RobotConfig& config);

    // set the speed of the wheel in radians/second
    void setWheelSpeed(unsigned int wheel_number, float speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) override;

    // se the position of the wheel in degrees.
    void setWheelPosition(unsigned int wheel_number, float position, PositionUnit position_unit = RADIAN) override;

    bool getWheelSpeed(unsigned int wheel_number, float &speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) override;

    bool getWheelPosition(unsigned int wheel_number, float &position, PositionUnit position_unit = RADIAN) override;

private:
    std::map<std::string, unsigned int> _wheel_remap;
    double _wheel_radius;
};
} // namespace wheel_driver_simulation

#endif