#ifndef KANGAROO_DRIVER
#define KANGAROO_DRIVER

#include <Kangaroo.h>
#include <Stream.h>
#include <robot_base/base_wheel_driver.h>
#include <robot_base/robot_config.h>
#include <map>
#include <memory>


namespace base_robot_controller
{
    class RobotConfig;
}

namespace kangaroo_driver
{

using namespace robot_base;

struct KangarooWheel
{
    std::shared_ptr<KangarooChannel> driving_wheel_ptr;
    std::shared_ptr<KangarooChannel> steering_wheel_ptr;
    double steering_wheel_zero, steering_wheel_scale;
    double driving_wheel_zero, driving_wheel_scale;
};

class KangarooDriver : public BaseWheelDriver
{
public:
    KangarooDriver() {}
    ~KangarooDriver();
    void initialize(const std::string& name, const RobotConfig& config);

    // set the speed of the wheel
    void setWheelSpeed(unsigned int wheel_number, float speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) override;

    // se the position of the wheel.
    void setWheelPosition(unsigned int wheel_number, float position, PositionUnit position_unit = RADIAN) override;

    bool getWheelSpeed(unsigned int wheel_number, float &speed, SpeedUnit speed_unit = RADIAN_PER_SECOND) override;

    bool getWheelPosition(unsigned int wheel_number, float &position, PositionUnit position_unit = RADIAN) override;

private:
    std::map<std::string, unsigned int> _wheel_remap;
    Stream _serial_port;
    std::shared_ptr<KangarooSerial> _kangaroo_serial_ptr;
    std::vector<double> _motor_calibration_zero, _motor_calibration_scale;
    std::vector<KangarooWheel> _wheels;
    RobotConfig _robot_config;
    int _max_steering_speed, _driving_speed_ramp;
    

};
} // namespace kangaroo_controller_driver

#endif