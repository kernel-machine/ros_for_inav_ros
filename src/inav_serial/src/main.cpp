#include <iostream>

#include "ros/ros.h"
#include "serial.h"
#include "std_msgs/Int32.h"
#include "custom_messages/Axis.h"
#include "string.h"

Serial * serial = NULL;
#define RECV_DATA

typedef struct{
    uint16_t connected;
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
    int16_t checksum;
    uint32_t roba;
} rosAxis_t;

#ifdef RECV_DATA
typedef struct {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
    int16_t throttle;
    
    uint32_t totalPacket;
    uint16_t badPacket;
    uint16_t rosModeEnable;
} rosReceivedAxis_t;

/*
rosReceivedAxis_t parseRosPackets(uint8_t * d){

    //Encode frame to get usefull data
    rosReceivedAxis_t rosData = {0};
    rosData.pitch =     *(int16_t*)(d);
    rosData.roll =      *(int16_t*)(d+2);
    rosData.yaw =       *(int16_t*)(d+4);
    rosData.throttle =  *(int16_t*)(d+6);

    return rosData;
}
*/
#endif

rosAxis_t axis;

void callback(custom_messages::Axis msg)
{
    axis.pitch = (msg.pitch/20)+1500;
    axis.roll = (msg.roll/20)+1500;
    axis.yaw = (msg.yaw/20)+1500;
    axis.throttle = (msg.throttle/20)+1500;
    axis.checksum =  axis.pitch ^ axis.roll ^ axis.yaw ^ axis.throttle;
    axis.connected = msg.isConnected;
    if(msg.isConnected==0)
        ROS_INFO("DISCONNECTED");

    //ROS_INFO("SENDED %d %d %d %d %u %d",axis.pitch, axis.roll, axis.throttle, axis.yaw, axis.connected, axis.checksum);   
    
    uint8_t * tmp = (uint8_t*)&axis;
    serial->writeData(tmp,sizeof(rosAxis_t));
    
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

    serial = new Serial(port_name);
    serial->setInterfaceAttribs(115200, 0);
    serial->setBlocking(true);

    if(serial->isOpen()){
        std::cout<<"Serial port opened successfully"<<std::endl;
    }
    else {
        std::cout<<"Error, cannot open the serial port"<<std::endl;
        return 0;
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("joystick_cmd", 1000, callback);

    #ifdef RECV_DATA
    rosReceivedAxis_t recvData = {0};
    const size_t recv_data_size = sizeof(rosReceivedAxis_t);
    #endif

    ros::Rate r(100);

    while(ros::ok()){

        ros::spinOnce();

        /*
        std::cout<<"spinn...";
        //serial->spin();
        std::cout<<"...ning"<<std::endl;
        */


        //std::cout<<"DATA AVAIABLE -> "<<serial->dataAvaiable()<<" <--"<<std::endl;
        
        #ifdef RECV_DATA
        
        //if(serial->dataAvaiable( ) > recv_data_size){
            serial->readData((uint8_t*)&recvData,recv_data_size);
            ROS_INFO("RECV p:%d r:%d t:%d y:%d tot:%u bad:%u ena:%u",
            recvData.pitch, recvData.roll, recvData.throttle, recvData.yaw,
            recvData.totalPacket, recvData.badPacket, recvData.rosModeEnable);
        //}       
        #endif
        
        

        //r.sleep();

    }
    delete serial;
    return 0;
}
