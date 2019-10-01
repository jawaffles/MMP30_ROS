#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG

#include <map>
#include <string>
#include <vector>

namespace robot_base
{

enum WheelType
{
    WHEEL_TYPE_DRIVING = 0,
    WHEEL_TYPE_DRIVING_STEERING,
    WHEEL_TYPE_PASSIVE
};


struct WheelCommand
{
    unsigned int wheel_id;
    float wheel_speed;
    float wheel_rotation;
};

enum SpeedUnit
{
    RADIAN_PER_SECOND = 0,
    ROTATION_PER_MINITE,
    RPM = ROTATION_PER_MINITE,
    DEGREE_PER_SECOND,
    METER_PER_SECOND
};

enum PositionUnit
{
    RADIAN = 0,
    DEGREE
};


struct WheelInfo
{
    WheelType type;
    unsigned int id;
    float position_x,  position_y;
    float speed_lower_limit, speed_upper_limit;
    float angle_lower_limit, angle_upper_limit;
    bool mirror_x, mirror_y; // this controls the wheel orientation
    double wheel_radius;
};

enum RobotType
{
    ROBOT_TYPE_2WD=0,
    ROBOT_TYPE_2WD_2WS,
    ROBOT_TYPE_4WD,
    ROBOT_TYPE_4WD_4WS,
    ROBOT_TYPE_CUSTOMIZE
};

class RobotConfig
{
public:
    RobotConfig();
    RobotConfig(const std::string& config_file);
    ~RobotConfig();
    
    void initMotorRemaps (std::map<std::string, unsigned int> &remap) const;
    // bool readConfigFromRobotDescription();
    bool readConfigFromFile(const std::string& config_file);
    WheelType getWheelType(unsigned int index ) const {return _wheels[index].type;}
    //unsigned int getWheelId(unsigned int index) const {return _wheels[index].id;}
    unsigned int getWheelId(unsigned int index) const {return index;}
    unsigned int getWheelCount() const {return _wheels.size(); }
    double getWheelRadius(unsigned int index) const {return _wheels[index].wheel_radius;}
    RobotType getRobotType() const { return _robot_type; }
    float getWheelPositionX(unsigned int index) const {return _wheels[index].position_x;}
    float getWheelPositionY(unsigned int index) const {return _wheels[index].position_y;}
	float getComX() const {return _COM_x;}
    float getComY() const {return _COM_y;}
    // float getWheelPositionZ(unsigned int index) const {return _wheels[index].position_z;}
private:
    RobotType _robot_type;
    unsigned int _wheel_count;
	double _COM_x, _COM_y;
    std::vector<WheelInfo> _wheels;
    std::string _robot_controller;
    std::string _wheel_driver;
    std::string _robot_name;

};

};

#endif
