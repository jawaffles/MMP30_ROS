#include "Kangaroo.h"
#include "Stream.h"
#include <basic_robot_controller/basic_robot_controller.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sstream>

using namespace std;


class DifferentialDrive
{
    public:
        
        //KangarooChannel Drive,Drive2 ,Turn, Turn2;
        KangarooChannel FR, FL, BR, BL ; // The Letters indicate which wheel FR = FRONT RIGHT WHEEL , BL = BACK LEFT WHEEL
        std::string port_name;
        Stream SerialPort;
        KangarooSerial K;

        

        double odom_x = 0; //The change in x position in an odometry frame
        double odom_y = 0; //The change in y position in an odometry frame
        double odom_ang = 0; //Change in theta (z axis) heading in odometry frame
        double del_Sr = 0; //Change in distance traveled for right wheels on rover
        double del_Sl = 0; //Change in distance traveled by left wheels on rover
        double prev_pos = 0;
        double drive_p = 0;
        double turn_p = 0;
            
        // For this constructor function for the Class Differential Drive (A function that activates as soon as instantiation) an input argument of serial port_name is accepted
        // As well as all the objects are defined for the two drive modules

        DifferentialDrive(std::string port_name): SerialPort(), K(SerialPort), FR(K, '2', 128), FL(K, '1', 128), BR(K, '2', 129),BL(K, '1', 129) {

            odom_x = 0;
            odom_y = 0;
            odom_ang = 0;

            // Stream SerialPort;
            // std::string port_name = "/dev/ttyUSB0";
            // KangarooSerial K(Stream SerialPort);
            // KangarooChannel Drive(KangarooSerial K, std::string T, int NUM);
            // KangarooChannel Turn(KangarooSerial K, std::string T, int NUM);
            // KangarooChannel Drive2(KangarooSerial K, std::string T, int NUM);
            // KangarooChannel Turn2(KangarooSerial K, std::string T, int NUM);

            // Stream SerialPort;
            
            
            std::cout << "DifferentialDrive Connecting!" << std::endl;

            try{
                
                std::cout << "Trying to Connect" << std::endl;

                SerialPort.begin(port_name, 100, B115200);
            }
            catch(std::exception& e){
                std::cout << "Could Not Connect: " << e.what() << std::endl;
                

            }
            std::cout << "Connection Tried" << std::endl;


            FR.start();
            FL.start();
            BR.start();
            BL.start();

            FR.s(0);
            FL.s(0);
            BR.s(0);
            BL.s(0);            
            
               
            

            std::cout << "Kangaroo System Initialized" << std::endl;

        }


        ~DifferentialDrive() {
            
            FR.s(0).wait();
            FL.s(0).wait();
            BR.s(0).wait();
            BL.s(0).wait();
           
            FR.powerDown();
            BR.powerDown();
            FL.powerDown();
            BL.powerDown();



        };


        void setRight(double ms)
        {
            int ticks = getTicks(ms);
            FR.s(ticks);
            BR.s(ticks);

        }

        void setLeft(double ms)
        {
            int ticks = getTicks(ms);
            
            FL.s(ticks);
            BL.s(ticks);

        }


        int16_t rightTicks()
        {
            // return round((FR.getP().value() + BR.getP().value()) / 2) ;  
            return round((FR.getP().value())) ; 


        }


        int16_t leftTicks()
        {
            // return round((FL.getP().value() + BL.getP().value()) / 2);
        
            return round((FL.getP().value() ));
        }


        int getTicks(double ms)
        {
            //Converts Meters per Seconds Velocity to Ticks per Second
            int ticks = ((ms) / (2*3.14 * radius)) * 45.96 * 500;
            // (Meter/Second)  * ( 1 / 2 * Pi * Radius) = # Revolutions per Second
            // # Rev/s * (Ticks / Rev) = Ticks per revolution

            return ticks;

        }




        void rvelCallback(const std_msgs::Float32::ConstPtr &vel)
        {
          

            setRight(vel->data);
   
        }


        void lvelCallback(const std_msgs::Float32::ConstPtr &vel)
        {
          

            setLeft(vel->data);
   
        }





        // void odomcalc()
        // {



    private:
        const double radius = .1016;  //Radius is 4 inches / .1016 meters
        const double length = .4191; //Length between wheels is 16 in / .4191 meters 

};







int main(int argc, char *argv[])
{
    

    ros::init(argc, argv, "MMP30");

    ros::NodeHandle mmp30;

    DifferentialDrive obj("/dev/ttyUSB0");

    

    ros::Subscriber subr = mmp30.subscribe("rwheel_vtarget", 1000, &DifferentialDrive::rvelCallback, &obj);

    ros::Subscriber subl = mmp30.subscribe("lwheel_vtarget",1000, &DifferentialDrive::lvelCallback, &obj);

    ros::Publisher rticks = mmp30.advertise<std_msgs::Int16>("rwheelticks",1000);
    ros::Publisher lticks = mmp30.advertise<std_msgs::Int16>("lwheelticks",1000);



    ros::Rate loop_rate(10);


    std_msgs::Int16 r_ticks;
    std_msgs::Int16 l_ticks;

    r_ticks.data = 0;
    l_ticks.data = 0;

    while(ros::ok())
    {

        r_ticks.data = obj.rightTicks();
        l_ticks.data = obj.leftTicks();

        rticks.publish(r_ticks);
        lticks.publish(l_ticks);

        ros::spinOnce();
        loop_rate.sleep();
        
    }

    delete &obj;

    return 0;

}





//Comments
    // Stream SerialPort;
    // std::string port_name = "/dev/ttyUSB0";
    // SerialPort.begin(port_name, 100, B115200);
    // KangarooSerial K(SerialPort);

    // KangarooChannel Drive(K, 'D', 128);
    // KangarooChannel Drive2(K, 'D', 129);
    // KangarooChannel Turn(K, 'T', 128);
    // KangarooChannel Turn2(K, 'T', 129);

    // Drive.start();
    // BL.start();

    // FL.start();
    // BR.start();
    
    // Drive.si(1000).wait();
    // BL.si(0);

    
    // FL.si(1000);
    // BR.si(0);


    // sleep(2);

   
    // return 0;

    // //KangarooError e=K1.start();
    //KangarooError e=K2.start();
    // //
    // if (e ==KANGAROO_NOT_HOMED)
    // {
    //      K2.home().wait(); 
    // }




    //     // std::cout << e <<"\n";
    //     // e=K2.start();
    //     // printf("start error = %d\n", e);
    //     // if (e ==KANGAROO_NOT_HOMED)
    //     // {
    //     //     K2.home().wait(); 
    //     // }
    //     K2.p(atoi(argv[1]),100).wait();

    //     // K2.p(500,300).wait();
    //     // K2.p(1200,300).wait();

        // int c=K2.getPI().value();
        // for (int i=0; i<10; i++)
        // {
        // 	printf("%d, valid=%d\n", K2.getP().value(), K2.getP().valid()); 
        // }
        

    // //    robot_base::BasicRobotController bc;
    // //    std::vector<robot_base::WheelCommand> wheel_command;
    // //    geometry_msgs::TwistPtr cmd_vel;
    // //    cmd_vel.reset(new geometry_msgs::Twist);
    // //    cmd_vel->linear.x=0.2;
    //  //   cmd_vel->angular.z=1;
    //  //   robot_base::RobotConfig config;
    //  //   config.readConfigFromFile("/home/rui/catkin_ws/src/flowerbot/robot_base/robot_config/2wd2ws_config.yaml");
    // //    bc.calculateWheelCommand(wheel_command, cmd_vel, config);
        
    //     // for (int i=0; i<200; i++)
    //     //     K1.s(i).wait();
    //     //while(1);
    //     // K2.units(1, 14);
    //     // K2.home().wait();
    //     // K2.p(0,5).wait();
    //     // K2.p(10, 5).wait();
    //     // K2.p(0,5).wait();


    //     // while (1)
    //     // {
    //     //    long minimum = K2.getMin().value();
    //     //     long maximum = K2.getMax().value();
    //     //     std::cout<<"min: "<<minimum<<", max: "<<maximum<<"\n";
    //     //     long speed = (maximum - minimum) / 10; // 1/10th of the range per second

    //     //     K2.s(speed).wait();
    //     //     sleep(5);

    //     //     K2.s(-speed).wait();
    //     //     sleep(5);
    //     // }



        // for (int i=0; i<5; i++)
        // {
            
        //     std::cout << "Drive Test Speed: " << std::endl;
        // 	Drive.s(i * 1000).wait();
        //     FL.s(i * 1000).wait();
        //     printf("%d \n", Drive.getS().value()); 
        //     sleep(2);
        // }

        //     for (int i=0; i>-5; i--)
        // {
            
        //     std::cout << "Drive Test Speed: " << std::endl;
        // 	Drive.s(i * 1000).wait();
        //     FL.s(i * 1000).wait();
        //     printf("%d \n", Drive.getS().value()); 
        //     sleep(2);
        // }
        
        
        // std::cout << "Drive Test Done" << std::endl;

        // Drive.s(0).wait();
        // FL.s(0).wait();

        // for (int j=0; j<5; j++)
        // {
            
        //     std::cout << "Turn Test Speed" << std::endl;
        // 	BL.s(j * 1000).wait();
        //     BR.s(j * 1000).wait();
        //     printf("%d \n", BL.getS().value()); 
        //     sleep(2);
        // }

        // std::cout << "Turn Test Done" << std::endl;