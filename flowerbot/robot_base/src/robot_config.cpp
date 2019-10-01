#include <robot_base/robot_config.h>
#include <robot_base/base_wheel_driver.h>
#include <robot_base/base_robot_controller.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <string>

namespace robot_base
{

RobotConfig::RobotConfig():_robot_type(ROBOT_TYPE_2WD),
_wheel_count(0),
_robot_controller("robot_base/BasicRobotController"),
_robot_name("flowerbot")
{
}

RobotConfig::RobotConfig(const std::string &config_file):_robot_type(ROBOT_TYPE_2WD),
_wheel_count(0),
_robot_controller("robot_base/BasicRobotController"),
_robot_name("flowerbot")
{
    RobotConfig();
    if (readConfigFromFile(config_file) == false)
    {
        throw "can't parse configuration file.";
        exit(1);
    }
}

RobotConfig::~RobotConfig()
{
}

void RobotConfig::initMotorRemaps(std::map<std::string, unsigned int> &remap) const
{
}

bool RobotConfig::readConfigFromFile(const std::string &config_file)
{
    std::string file_extension = config_file.substr(config_file.find_last_of(".") + 1);
    if (file_extension == "yaml")
    {
        YAML::Node config = YAML::LoadFile(config_file);
        _robot_name = config["name"].as<std::string>();
        _robot_controller = config["robot_controller"].as<std::string>();
        _wheel_driver = config["wheel_driver"].as<std::string>();
		_wheel_count = config["wheel_count"].as<int>();
		_COM_x = config["COM_x"].as<double>(0);
        _COM_y = config["COM_y"].as<double>(0);
        WheelInfo new_wheel;
        YAML::Node wheel_node = config["wheel_description"];
        if (wheel_node.IsNull())
        {
            throw "no wheel description found.";
            return false;
        }
        if (wheel_node.IsNull())
        {
            throw "Can't find wheel description.";
            return false;
        }
        if (!wheel_node.IsMap())
        {
            throw "wheel descritpion should be a map.";
            return false;
        }
        if (wheel_node.IsMap())
        {
/*
            for (YAML::iterator wheel = wheel_node.begin(); wheel != wheel_node.end(); ++wheel)
            {
                YAML::Node wheel_value = wheel->second;
				printf("===================== \n");
                new_wheel.id = wheel_value["id"].as<int>();
				printf("id = %d \n", new_wheel.id);
                new_wheel.type = (WheelType)(wheel_value["type"].as<int>());
				printf("type = %d\n", new_wheel.type);
                new_wheel.position_x = wheel_value["x"].as<float>();
				printf("position_x = %f\n", new_wheel.position_x);
                new_wheel.position_y = wheel_value["y"].as<float>();
                new_wheel.speed_lower_limit = wheel_value["speed_lower_limit"].as<float>(-0.4);
                new_wheel.speed_upper_limit = wheel_value["speed_upper_limit"].as<float>(0.4);
                new_wheel.angle_lower_limit = wheel_value["angle_lower_limit"].as<float>(-1.5708);
                new_wheel.angle_upper_limit = wheel_value["angle_upper_limit"].as<float>(1.5708);
                new_wheel.mirror_x = wheel_value["mirror_x"].as<bool>(false);
                new_wheel.mirror_y = wheel_value["mirror_y"].as<bool>(false);
                new_wheel.wheel_radius = wheel_value["wheel_radius"].as<double>();
				printf("wheel_radius = %f\n", new_wheel.wheel_radius);
				
                _wheels.push_back(new_wheel);
                
            }
*/
            for (int iWheel = 0; iWheel < _wheel_count; ++iWheel)
            {
                YAML::Node wheel_value = wheel_node["wheel"+std::to_string(iWheel)];
				printf("===================== \n");
                new_wheel.id = wheel_value["id"].as<int>();
				printf("id = %d \n", new_wheel.id);
                new_wheel.type = (WheelType)(wheel_value["type"].as<int>());
				printf("type = %d\n", new_wheel.type);
                new_wheel.position_x = wheel_value["x"].as<float>();
				printf("position_x = %f\n", new_wheel.position_x);
                new_wheel.position_y = wheel_value["y"].as<float>();
				printf("position_y = %f\n", new_wheel.position_y);
                new_wheel.speed_lower_limit = wheel_value["speed_lower_limit"].as<float>(-0.4);
                new_wheel.speed_upper_limit = wheel_value["speed_upper_limit"].as<float>(0.4);
                new_wheel.angle_lower_limit = wheel_value["angle_lower_limit"].as<float>(-1.5708);
                new_wheel.angle_upper_limit = wheel_value["angle_upper_limit"].as<float>(1.5708);
                new_wheel.mirror_x = wheel_value["mirror_x"].as<bool>(false);
                new_wheel.mirror_y = wheel_value["mirror_y"].as<bool>(false);
                new_wheel.wheel_radius = wheel_value["wheel_radius"].as<double>();
				printf("wheel_radius = %f\n", new_wheel.wheel_radius);
				
                _wheels.push_back(new_wheel);
                
            }
            _wheel_count=_wheels.size();
            return true;
        }
    }
    else
    {
        throw "Unsupport file format: " + file_extension;
        return false;
    }
}

} // namespace robot_base
