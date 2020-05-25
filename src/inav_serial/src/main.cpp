#include <iostream>

#include "ros/ros.h"
#include "serial.h"
#include "std_msgs/Int32.h"
#include "custom_messages/Axis.h"


void callback(const custom_messages::Axis::ConstPtr& msg)
{
    ROS_INFO("I heard: %u",msg->pitch);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "serial_node");

    std::cout<<"Serial opening"<<std::endl;

    const char * port_name;

    if (argc > 2 && strcmp("--input", argv[1]) == 0)
    {
        port_name = argv[2];
    }
    else
        port_name = "/dev/ttyUSB0";

    Serial * serial = new Serial(port_name);
    serial->serialSetInterfaceAttribs(115200, 0);
    serial->serialSetBlocking(true);

    if(serial->isOpen()){
        std::cout<<"Serial port opened successfully"<<std::endl;
    }
    else {
        std::cout<<"Error, cannot open the serial port"<<std::endl;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joystick_cmd", 1000, callback);
    ros::spin();
    return 0;
}