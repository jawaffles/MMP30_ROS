#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <gps_common/conversions.h>

#include <sstream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <endian.h>
#include <math.h>
#include <termios.h>
#include <sys/ioctl.h>

#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#define BUFFER_SIZE 200
#define ROBOT_PORT "/dev/ttyACM1"

int port;

// strcuture for imu data
struct ImuMsg
{
	uint64_t timeStartup;
	float yaw, pitch, roll;
	float angVelX, angVelY, angVelZ;
	float acclX, acclY, acclZ;
	uint32_t vpeStatus;
};

// structure for gps data
struct GpsMsg
{
	double second;
	double latitude;
	double longitude;
	double altitude;
	double hdop;
	unsigned char numSat;
	unsigned char fix;

};

// structure for robot speed from encoder
struct speedMsg
{
	float forwardSpeed;
	float turningSpeed;
};

// Calculates the 16-bit CRC for the given ASCII or binary message.
uint16_t calculateCRC(unsigned char data[], uint16_t crcInit, unsigned int length)
{
	unsigned int i;
	uint16_t crc = crcInit;
	for (i = 0; i < length; i++){
		crc = (crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (crc & 0x00ff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}
	return crc;
}

// receive cmd_vel and send to the robot
// this data string is formated as 
// 0xFA + bytes of cmd_vel + 2 bytes crc
void cmdVelReceived(const geometry_msgs::Twist cmd_vel)
{
	ROS_INFO("cmd: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);
	static unsigned char sentBuffer[150];

	sentBuffer[0] = 0xFA;
	memcpy(sentBuffer + 1, &cmd_vel, sizeof(geometry_msgs::Twist));
	uint16_t crc = calculateCRC(sentBuffer + 1, 0, sizeof(geometry_msgs::Twist));
	sentBuffer[sizeof(geometry_msgs::Twist) + 2] = crc;
	sentBuffer[sizeof(geometry_msgs::Twist) + 1] = crc >> 8;

	int wtrz = write(port, sentBuffer, sizeof(geometry_msgs::Twist) + 3);
	if (wtrz < (sizeof(geometry_msgs::Twist) + 3))
	{
		ROS_WARN_NAMED("robot", "Can't send command to robot.");
	}
}


// initialize the serial port
int initRobotPort(const char * port)
{
	int ret;
	if ((ret = open(port, O_RDWR | O_NONBLOCK)) < 0)
	{
		return ret;
	}
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(ret, &tty) != 0)
	{
		ROS_ERROR_STREAM_NAMED("Robot", "from tcgetattr: " << strerror(errno));
		return -1;
	}
	/* Set Baud Rate */
	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	/* Setting other Port Stuff */
	tty.c_cflag &= ~PARENB;		// Make 8n1
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;	// no flow control
	tty.c_lflag = 0;			// no signaling chars, no echo, no canonical processing
	tty.c_oflag = 0;			// no remapping, no delays
	tty.c_cc[VMIN] = 0;			// read doesn't block
	tty.c_cc[VTIME] = 0;		// 0.5 seconds read timeout

	tty.c_cflag |= CREAD | CLOCAL;						// turn on READ & ignore ctrl lines
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);				// turn off s/w flow ctrl
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);		// make raw
	tty.c_oflag &= ~OPOST;								// make raw

	tcflush(ret, TCIFLUSH);

	if (tcsetattr(ret, TCSANOW, &tty) != 0)
	{
		//ROS_ERROR_STREAM_NAMED("Robot", "from tcsetattr: " << strerror(errno));
		return -1;
	}
	return ret;
}

// read bytes from the serial port until characters detected and save the data to buf
// return the number of receveived bytes
int readBytesUntil(int port, unsigned char *buf, unsigned char *characters, int char_len, int len)
{
	unsigned char tmp[1];
	unsigned char tmp1[25];
	unsigned char tmp2[25];
	int i = 0;
	memset(tmp1, 0, char_len);
	for (i = 0; i < len; i++)
	{
		if (read(port, tmp, 1) == 1)
		{
			memcpy(tmp2, tmp1 + 1, char_len - 1);
			tmp2[char_len - 1] = tmp[0];
			memcpy(tmp1, tmp2, char_len);
			if (memcmp(tmp1, characters, char_len) == 0)
			{
				return i + char_len;
			}
		}
		else
			return -1;
	}
	return -2;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		printf("usage: robot_node serial_port\n");
		return 0;
	}

	ros::init(argc, argv, "robot");
	ros::NodeHandle nodeHandle;
	// subscribe cmd_vel topic
	ros::Subscriber sub = nodeHandle.subscribe("/cmd_vel", 1, cmdVelReceived);

	// publisher for /odometry/gps topic
	ros::Publisher gps_odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("odometry/gps", 1);
	// publisher for /fix topic
	ros::Publisher gps_fix_pub = nodeHandle.advertise<sensor_msgs::NavSatFix>("fix", 1);
	// publisher for /imu/data topic
	ros::Publisher imu_pub = nodeHandle.advertise<sensor_msgs::Imu>("imu/data", 1);
	// publisher for /odometry/wheel topic
	ros::Publisher wheel_odom_pub = nodeHandle.advertise<nav_msgs::Odometry>("odometry/wheel", 1);
	// publisher for /battery topic
	ros::Publisher battery_pub = nodeHandle.advertise<sensor_msgs::BatteryState>("battery", 1);
	tf::TransformBroadcaster odom_broadcaster;
	// update rate is 40Hz
	ros::Rate rate(40.0);

	double simulationStepTime = 0.025;
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = current_time;

	bool isImuSimulationMode = false, isGpsSimulationMode = false;
	double rollSimulation = 0, pitchSimulation = 0, yawSimulation = 0;

	geometry_msgs::Pose poseSimulation;
	nav_msgs::Odometry wheelOdom;
	nav_msgs::Odometry gpsOdom;
	sensor_msgs::Imu imu;
	sensor_msgs::NavSatFix fix;
	sensor_msgs::BatteryState battery;

	wheelOdom.header.frame_id = "odom";
	wheelOdom.child_frame_id = "base_footprint";
	gpsOdom.header.frame_id = "map";
	gpsOdom.child_frame_id = "gps_link";
	imu.header.frame_id = "base_footprint";

	battery.header.frame_id = "base_footprint";
	battery.current = NAN;
	battery.charge = NAN;
	battery.capacity = 3;
	battery.design_capacity = 3;
	battery.percentage = NAN;
	battery.power_supply_status = battery.POWER_SUPPLY_STATUS_UNKNOWN;
	battery.power_supply_health = battery.POWER_SUPPLY_HEALTH_UNKNOWN;
	battery.power_supply_technology = battery.POWER_SUPPLY_TECHNOLOGY_NICD;
	battery.present = true;
	battery.location = "slot1";
	battery.serial_number = "1";

	// read parameters
	ros::NodeHandle privateNodeHandle("~");
	privateNodeHandle.getParam("imu_simulation", isImuSimulationMode);
	privateNodeHandle.getParam("gps_simulation", isGpsSimulationMode);

	// initialize simulated gps and imu
	if (isGpsSimulationMode)
	{
		privateNodeHandle.getParam("location_origin_x", wheelOdom.pose.pose.position.x);
		privateNodeHandle.getParam("location_origin_y", wheelOdom.pose.pose.position.y);
		privateNodeHandle.getParam("location_origin_z", wheelOdom.pose.pose.position.z);
	}
	if (isImuSimulationMode)
	{
		privateNodeHandle.getParam("orientation_origin_x", rollSimulation);
		privateNodeHandle.getParam("orientation_origin_y", pitchSimulation);
		privateNodeHandle.getParam("orientation_origin_z", yawSimulation);
	}


	unsigned char recBuffer[BUFFER_SIZE];
	unsigned char header[] = { 0xFA, 0xFF };
	struct ImuMsg robotIMU;
	struct GpsMsg robotGPS;
	struct speedMsg robotSpeed;
	float batteryVoltage;

	if ((port = initRobotPort(argv[1])) < 0)
	{
		ROS_ERROR_STREAM_NAMED("robot", "Can't open robot port: " << argv[1]);
		return 1;
	}
	memset(recBuffer, 0, BUFFER_SIZE);



	while (nodeHandle.ok())
	{
		double ttt = ros::Time::now().toSec();

		int bytesAvailable;
		bool imuValid = false;
		bool gpsValid = false;
		bool wheelOdomValid = false;
		bool batteryValid = true;
		ioctl(port, FIONREAD, &bytesAvailable);
		ROS_DEBUG_STREAM_NAMED("robot", "bytesAvailable: " << bytesAvailable);


		// the robot sends data to the raspberry pi using the following format
		// starts with header 0xFA, 0xFF
		// follows with 2 bytes indiciating the length of the data
		// follows with 1 byte to indicate where it contains gps data and imu data
		// follows with the data bytes
		// follows with 2 bytes of crc

		// first read the all the bytes in the serial port buffer until the header
		// this is to clear the data that was not read last time
		int offset = readBytesUntil(port, recBuffer, header, 2, bytesAvailable);
		ROS_DEBUG_STREAM_NAMED("robot", "offset: " << offset);
		if (offset >= 0)
		{
			// read 2 bytes to get the length of the data
			if (read(port, recBuffer, 2) == 2)
			{
				int bytes2Read = 0;
				memcpy((unsigned char*)&bytes2Read, recBuffer, 2);

				ROS_DEBUG_STREAM_NAMED("robot", "bytes2Read: " << bytes2Read);
				int bytesHasRead = 0;
				int readTry = 0;

				// read all the data from the port
				// due to the speed of the raspberry pi, it can't read all the data from the port at once
				// therefore a while loop is used here
				while (bytesHasRead < bytes2Read)
				{
					bytesHasRead += read(port, recBuffer + bytesHasRead, bytes2Read - bytesHasRead);
				}
				// calcualte the crc 
				uint16_t crc = calculateCRC(recBuffer, 0, bytes2Read - 2);
				uint16_t crcRec = recBuffer[bytes2Read - 1];
				crcRec = crcRec << 8 | recBuffer[bytes2Read - 2];

				// compare with the crc received, if they match, the data recevied this time is correct
				if (crc == crcRec)
				{
					unsigned char *p = recBuffer;
					p++;
					if (recBuffer[0] & 0x01)
					{
						memcpy(&robotGPS, p, sizeof(GpsMsg));
						gpsValid = true;
						p += sizeof(GpsMsg);
					}
					if (recBuffer[0] & 0x02)
					{
						memcpy(&robotIMU, p, sizeof(ImuMsg));
						imuValid = true;
						p += sizeof(ImuMsg);
					}
					memcpy((unsigned char*)&robotSpeed, p, sizeof(speedMsg));
					wheelOdomValid = true;
					p += sizeof(speedMsg);
					memcpy((unsigned char*)&batteryVoltage, p, sizeof(float));
					batteryValid = true;
					//printf("%f,%f\n", robotSpeed.forwardSpeed, robotSpeed.turningSpeed);
					ROS_DEBUG_NAMED("robot", "Received a message");
				}
				else
					ROS_DEBUG_NAMED("robot", "CRC is incorrect.");
			}

		}
		last_time = current_time;
		current_time = ros::Time::now();

		// if received battery voltage
		if (batteryValid)
		{
			battery.header.stamp = current_time;
			battery.voltage = batteryVoltage;
			battery_pub.publish(battery);
		}

		// if received wheel dometry
		if (wheelOdomValid) // publish wheel odometry
		{

			wheelOdom.header.stamp = current_time;

			if (isGpsSimulationMode)
			{
				wheelOdom.pose.pose.position.x += (robotSpeed.forwardSpeed * cos(yawSimulation)) * simulationStepTime;
				wheelOdom.pose.pose.position.y += (robotSpeed.forwardSpeed * sin(yawSimulation)) * simulationStepTime;
				wheelOdom.pose.pose.position.z = 0;
			}
			else
			{
				wheelOdom.pose.pose.position.x = 0;
				wheelOdom.pose.pose.position.y = 0;
				wheelOdom.pose.pose.position.z = 0;//robotGPS.altitude;
			}
			//printf("%f,%f,%f\n", yawSimulation, wheelOdom.pose.pose.position.x, wheelOdom.pose.pose.position.y);
			
			if (isImuSimulationMode)
			{
				// update the simulated yaw angle
				yawSimulation += robotSpeed.turningSpeed*simulationStepTime; 
				if (yawSimulation > 2 * M_PI)
					yawSimulation = yawSimulation - 2 * M_PI;
				if (yawSimulation < -2 * M_PI)
					yawSimulation = yawSimulation + 2 * M_PI;
			}

			wheelOdom.pose.pose.orientation.x = 1;               // identity quaternion
			wheelOdom.pose.pose.orientation.y = 0;               // identity quaternion
			wheelOdom.pose.pose.orientation.z = 0;               // identity quaternion
			wheelOdom.pose.pose.orientation.w = 0;               // identity quaternion

			wheelOdom.pose.covariance[0] = 0.01;
			wheelOdom.pose.covariance[7] = 0.01;
			wheelOdom.pose.covariance[14] = 0.01;
			wheelOdom.pose.covariance[21] = 99999;
			wheelOdom.pose.covariance[28] = 99999;
			wheelOdom.pose.covariance[35] = 0.001;

			wheelOdom.twist.twist.linear.x = robotSpeed.forwardSpeed;
			wheelOdom.twist.twist.linear.y = 0;
			wheelOdom.twist.twist.linear.z = 0;
			wheelOdom.twist.twist.angular.x = 0;
			wheelOdom.twist.twist.angular.y = 0;
			wheelOdom.twist.twist.angular.z = robotSpeed.turningSpeed;
			wheelOdom.twist.covariance[0] = 0.0001;
			wheelOdom.twist.covariance[7] = 0.0001;
			wheelOdom.twist.covariance[14] = 0.0001;
			wheelOdom.twist.covariance[21] = 0.00001;
			wheelOdom.twist.covariance[28] = 0.00001;
			wheelOdom.twist.covariance[35] = 0.0001;
			wheel_odom_pub.publish(wheelOdom);
		}

		// if received imu data or the imu data is simulated
		if (imuValid || isImuSimulationMode)
		{
			//ROS_INFO_STREAM_NAMED("robot", "Orientation:"<<robotIMU.yaw<<","<<robotIMU.pitch<<","<<robotIMU.roll);
			//ROS_INFO_STREAM_NAMED("robot", "Acceleration:"<<robotIMU.acclX<<","<<robotIMU.acclY<<","<<robotIMU.acclZ);
			imu.header.stamp = current_time;

			if (isImuSimulationMode)
			{
				imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(rollSimulation, pitchSimulation, yawSimulation);
			}
			else
			{
				imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(robotIMU.roll*M_PI / 180, robotIMU.pitch*M_PI / 180, robotIMU.yaw*M_PI / 180);
			}

			imu.orientation_covariance[0] = 0.0003;
			imu.orientation_covariance[4] = 0.0003;
			imu.orientation_covariance[8] = 0.001;
			if (isImuSimulationMode)
			{
				imu.angular_velocity.x = 0;
				imu.angular_velocity.y = 0;
				imu.angular_velocity.z = wheelOdom.twist.twist.angular.z;
			}
			else
			{
				imu.angular_velocity.x = robotIMU.angVelX;
				imu.angular_velocity.y = robotIMU.angVelY;
				imu.angular_velocity.z = robotIMU.angVelZ;
			}
			imu.angular_velocity_covariance[0] = 0.003;
			imu.angular_velocity_covariance[4] = 0.003;
			imu.angular_velocity_covariance[8] = 0.003;
			if (isImuSimulationMode)
			{
				imu.linear_acceleration.x = 0;
				imu.linear_acceleration.y = 0;
				imu.linear_acceleration.z = 0;
				imu.linear_acceleration_covariance[0] = 99999;
				imu.linear_acceleration_covariance[0] = 99999;
				imu.linear_acceleration_covariance[0] = 99999;
			}
			else
			{
				imu.linear_acceleration.x = robotIMU.acclX;
				imu.linear_acceleration.y = robotIMU.acclY;
				imu.linear_acceleration.z = robotIMU.acclZ;
				imu.linear_acceleration_covariance[0] = 0.01;
				imu.linear_acceleration_covariance[0] = 0.01;
				imu.linear_acceleration_covariance[0] = 0.01;
			}

			imu_pub.publish(imu);
		}

		if (gpsValid || isGpsSimulationMode)
		{
			//ROS_INFO_STREAM_NAMED("received string", buffer);
			//memcpy(&robot_status, recBuffer+2, sizeof(RobotStatus);

			//ROS_DEBUG_STREAM_NAMED("robot", "Orientation:"<<robot_status.yaw<<","<<robot_status.pitch<<","<<robot_status.roll);
			//ROS_DEBUG_STREAM_NAMED("robot", "Acceleration:"<<robot_status.acclX<<","<<robot_status.acclY<<","<<robot_status.acclZ);


			gpsOdom.header.stamp = current_time;

			if (isGpsSimulationMode)
			{
				fix.latitude = wheelOdom.pose.pose.position.x;
				fix.longitude = wheelOdom.pose.pose.position.y;
				fix.altitude = wheelOdom.pose.pose.position.z;
				gpsOdom.pose.pose.position.x = wheelOdom.pose.pose.position.x;
				gpsOdom.pose.pose.position.y = wheelOdom.pose.pose.position.y;
				gpsOdom.pose.pose.position.z = wheelOdom.pose.pose.position.z;
			}
			else
			{
				fix.latitude = robotGPS.latitude;
				fix.longitude = robotGPS.longitude;
				fix.altitude = robotGPS.altitude;
				gps_common::UTM(robotGPS.latitude, robotGPS.longitude, &gpsOdom.pose.pose.position.x, &gpsOdom.pose.pose.position.y);
				gpsOdom.pose.pose.position.z = robotGPS.altitude;
			}

			gpsOdom.pose.pose.orientation.x = 1;               // identity quaternion
			gpsOdom.pose.pose.orientation.y = 0;             // identity quaternion
			gpsOdom.pose.pose.orientation.z = 0;               // identity quaternion
			gpsOdom.pose.pose.orientation.w = 0;               // identity quaternion
			if (isGpsSimulationMode)
			{
				robotGPS.fix = 0x04;
			}

			// assign pose covariance based on the quality of the gps signal
			switch (robotGPS.fix)
			{
			case 0x00:

				fix.status.service = fix.status.STATUS_NO_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_UNKNOWN;
				gpsOdom.pose.covariance[0] = 99999;
				gpsOdom.pose.covariance[7] = 99999;
				gpsOdom.pose.covariance[14] = 99999;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			case 0x01:
				fix.position_covariance[0] = 2.25;
				fix.position_covariance[4] = 2.25;
				fix.position_covariance[8] = 2.25;
				fix.status.service = fix.status.SERVICE_GPS;
				fix.status.service = fix.status.STATUS_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
				gpsOdom.pose.covariance[0] = 2.25;
				gpsOdom.pose.covariance[7] = 2.25;
				gpsOdom.pose.covariance[14] = 2.25;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			case 0x02:
				fix.position_covariance[0] = 0.25;
				fix.position_covariance[4] = 0.25;
				fix.position_covariance[8] = 0.25;
				fix.status.service = fix.status.STATUS_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
				gpsOdom.pose.covariance[0] = 2.25;
				gpsOdom.pose.covariance[7] = 2.25;
				gpsOdom.pose.covariance[14] = 2.25;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			case 0x04:
				fix.position_covariance[0] = 0.0001;
				fix.position_covariance[4] = 0.0001;
				fix.position_covariance[8] = 0.0001;
				fix.status.service = fix.status.STATUS_GBAS_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
				gpsOdom.pose.covariance[0] = 0.0001;
				gpsOdom.pose.covariance[7] = 0.0001;
				gpsOdom.pose.covariance[14] = 0.0001;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			case 0x05:
				fix.position_covariance[0] = 1;
				fix.position_covariance[4] = 1;
				fix.position_covariance[8] = 1;
				fix.status.service = fix.status.STATUS_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_DIAGONAL_KNOWN;
				gpsOdom.pose.covariance[0] = 1;
				gpsOdom.pose.covariance[7] = 1;
				gpsOdom.pose.covariance[14] = 1;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			default:
				fix.status.service = fix.status.STATUS_NO_FIX;
				fix.position_covariance_type = fix.COVARIANCE_TYPE_UNKNOWN;
				gpsOdom.pose.covariance[0] = 99999;
				gpsOdom.pose.covariance[7] = 99999;
				gpsOdom.pose.covariance[14] = 99999;
				gpsOdom.pose.covariance[21] = 99999;
				gpsOdom.pose.covariance[28] = 99999;
				gpsOdom.pose.covariance[35] = 99999;
				break;
			}

			// there is no twist for gps, so assign the covariance to a very large value
			gpsOdom.twist.covariance[0] = 9999;
			gpsOdom.twist.covariance[7] = 9999;
			gpsOdom.twist.covariance[14] = 9999;
			gpsOdom.twist.covariance[21] = 9999;
			gpsOdom.twist.covariance[28] = 9999;
			gpsOdom.twist.covariance[35] = 9999;
			gps_odom_pub.publish(gpsOdom);
			ROS_DEBUG_NAMED("robot", "publish gpsOdom");

			fix.header.stamp = current_time;
			fix.header.frame_id = "map";

			gps_fix_pub.publish(fix);
		}

		ros::spinOnce();
		rate.sleep();
	}
	close(port);
	return 0;
}

