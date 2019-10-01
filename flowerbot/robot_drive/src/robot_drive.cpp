#include <robot_drive/robot_drive.h>
#include <robot_base/robot_config.h>
#include <robot_base/base_robot_controller.h>



namespace robot_drive
{

RobotDriverRos::RobotDriverRos(const ros::NodeHandle &pnh, const std::string &ns) : pnh_(pnh),
                                                                                    _wheel_driver(NULL),
                                                                                    _robot_controller(NULL),
                                                                                    _wheel_driver_loader("robot_base", "robot_base::BaseWheelDriver"),
                                                                                    _robot_controller_loader("robot_base", "robot_base::BaseRobotController"),
                                                                                    _robot_config()
{
    
    std::string wheel_driver, robot_controller;
    pnh_.param<std::string>("wheel_driver", wheel_driver, "kangaroo_driver/KangarooDriver");
    pnh_.param<std::string>("robot_controller", robot_controller, "robot_base/BasicRobotController");
    pnh_.param<std::string>("robot_config_file", _robot_config_file, "/home/rui/catkin_ws/src/flowerbot/robot_base/robot_config/2wd2ws_config.yaml");

    double odom_publish_rate;
    pnh_.param<double>("odom_publish_rate", odom_publish_rate, 50);
    ROS_INFO_STREAM("odom_publish_rate is "<<odom_publish_rate);

    _rate_ptr.reset(new ros::Rate(odom_publish_rate));
    pnh_.param<bool>("simulation", _simulation, false);

    _robot_config.readConfigFromFile(_robot_config_file);


    // if (_simulation)
    // {
    //     ROS_INFO("Wait for gazebo services.");
    //     _gazebo_link_state_client_ptr.reset(&pnh_.serviceClient<gazebo_msgs::SetLinkState>("gazebo/set_link_state", true));
    //     if (_gazebo_link_state_client_ptr->waitForExistence(ros::Duration(10)) == false)
    //     {
    //         ROS_ERROR("Gazebo services are not available. Can't start gazebo simulation.");

    //     }
    // }

    // initialize joint state
    int joint_count=0;
    for (int i=0; i<_robot_config.getWheelCount(); i++)
    {
        //if (_robot_config.getWheelType(i) != robot_base::WHEEL_TYPE_PASSIVE)
        {
            _joint_states.name.push_back("wheel"+std::to_string(i)+"_driving_joint");
            joint_count++;
            if (_robot_config.getWheelType(i) == robot_base::WHEEL_TYPE_DRIVING_STEERING)
            {
                _joint_states.name.push_back("wheel"+std::to_string(i)+"_steering_joint");
                joint_count++;
            }
        }
    }
    _joint_states.position.resize(joint_count, 0.0);
    _joint_states.velocity.resize(joint_count, 0.0);
    _joint_states.effort.resize(joint_count, 0.0);
    

    ROS_INFO("initialize the robot controller");
    //initialize the robot controller
    try
    {
        _robot_controller = _robot_controller_loader.createInstance(robot_controller);
        _robot_controller->initialize(pnh_.getNamespace() + "/" + _robot_controller_loader.getName(robot_controller));
    }
    catch (const pluginlib::PluginlibException &ex)
    {
        ROS_FATAL("Failed to create the %s controller, are you sure it is properly registered and that the containing library is built? Exception: %s", wheel_driver.c_str(), ex.what());
        exit(1);
    }

    ROS_INFO("initialize the wheel driver");
    //initialize the wheel driver
    try
    {
        _wheel_driver = _wheel_driver_loader.createInstance(wheel_driver);
        _wheel_driver->initialize(pnh_.getNamespace() + "/" + _wheel_driver_loader.getName(wheel_driver), _robot_config);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
        ROS_FATAL("Failed to create the %s driver, are you sure it is properly registered and that the containing library is built? Exception: %s", wheel_driver.c_str(), ex.what());
        exit(1);
    }

    _cmd_vel_sub = pnh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &RobotDriverRos::cmdVelCallback, this);
    _odometry_pub = pnh_.advertise<nav_msgs::Odometry>("/odom", 100);
    _joint_state_publisher = pnh_.advertise <sensor_msgs::JointState>("joint_states",100);
	ROS_INFO_NAMED("robot_drive", "Finish initialization");
}

RobotDriverRos::~RobotDriverRos()
{
}

void RobotDriverRos::cmdVelCallback(const geometry_msgs::TwistConstPtr &cmd_vel)
{
	//ROS_INFO_STREAM("received cmd_vel: x=" << cmd_vel->linear.x <<", y="<<cmd_vel->linear.y << ", angle_z=" << cmd_vel->angular.z);
    std::vector<robot_base::WheelCommand> wheel_cmd;
    _robot_controller->calculateWheelCommand(wheel_cmd, cmd_vel, _robot_config);
    
    for (int i = 0; i < _robot_config.getWheelCount(); i++)
    {
        if (_robot_config.getWheelType(i) != robot_base::WHEEL_TYPE_PASSIVE)
        {
            _wheel_driver->setWheelSpeed(wheel_cmd[i].wheel_id, wheel_cmd[i].wheel_speed);
            if (_robot_config.getWheelType(i) == robot_base::WHEEL_TYPE_DRIVING_STEERING)
            {
                _wheel_driver->setWheelPosition(wheel_cmd[i].wheel_id, wheel_cmd[i].wheel_rotation);
            }
        }
    }
}
void RobotDriverRos::publishOdometry()
{
    std::vector<robot_base::WheelCommand> wheel_cmd;
    int joint_count=0;
    _joint_states.header.stamp=ros::Time::now();
    for (unsigned int i = 0; i < _robot_config.getWheelCount(); i++)
    {
		robot_base::WheelCommand wc;	
        
        wc.wheel_id = _robot_config.getWheelId(i);
        if (_robot_config.getWheelType(i) != robot_base::WHEEL_TYPE_PASSIVE)
        {
            float speed;
            _wheel_driver->getWheelSpeed(wc.wheel_id, speed);
            wc.wheel_speed = speed;
                        // change joint states
            _joint_states.velocity[joint_count++]=speed;
			
            if (_robot_config.getWheelType(i) == robot_base::WHEEL_TYPE_DRIVING_STEERING)
            {
                float position;
                _wheel_driver->getWheelPosition(wc.wheel_id, position);
                wc.wheel_rotation = position;
                _joint_states.position[joint_count++]=position;
            }
        }
        else
        {
            _joint_states.velocity[joint_count++]=0;
        }
        
        wheel_cmd.push_back(wc);
    }
    _robot_controller->calculateOdometry(_odom, wheel_cmd, _robot_config);
    _odometry_pub.publish(_odom);
    _joint_state_publisher.publish(_joint_states);
}

void RobotDriverRos::spin()
{
    while (ros::ok())
    {
        publishOdometry();
        _rate_ptr->sleep();
        ros::spinOnce();
    }
}

} // namespace robot_drive
