#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "custom_messages/Axis.h"
#include "lib/socket.h"
#include "lib/JsAxis.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

Socket * s = NULL;
custom_messages::Axis msg;
ros::Publisher pub;


void callback(const uint8_t * data, size_t size){
    
    std::string js ((const char*)data);
    size_t delimiter_pos = js.find("|");

    while(delimiter_pos!=std::string::npos){
        
        js = js.substr(delimiter_pos+1);

        const size_t p_pos = js.find("p:");
        const size_t r_pos = js.find("r:");
        const size_t t_pos = js.find("t:");
        const size_t y_pos = js.find("y:");

        if(p_pos==std::string::npos || 
        r_pos==std::string::npos || 
        t_pos==std::string::npos || 
        y_pos==std::string::npos)break;

        const std::string p_str = js.substr(p_pos,r_pos);
        const std::string r_str = js.substr(r_pos,t_pos);
        const std::string t_str = js.substr(t_pos,y_pos);
        const std::string y_str = js.substr(y_pos);

        const int p_level = std::stoi(p_str.substr(2));
        const int r_level = std::stoi(r_str.substr(2));
        const int t_level = std::stoi(t_str.substr(2));
        const int y_level = std::stoi(y_str.substr(2));

        std::cout<<p_level<<" "<<r_level<<" "<<t_level<<" "<<y_level<<std::endl;
        msg.pitch=p_level;
        msg.roll=r_level;
        msg.throttle=t_level;
        msg.yaw=y_level;
        msg.isConnected=1;
        pub.publish(msg);
        
        delimiter_pos = js.find("|");
    }
}

void disconnectedCallback(){
    msg.pitch=0;
    msg.roll=0;
    msg.throttle=1000;
    msg.yaw=0;
    msg.isConnected=0;
    pub.publish(msg);
}

void signalHandler(int sig){
    std::cout<<"signal "<<sig<<std::endl;
    delete s;
    exit(1);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joystick_receiver");
    ros::NodeHandle n;

    pub = n.advertise<custom_messages::Axis>("joystick_cmd", 1000);

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    s = new Socket(8098);
    if(s->isOpen()){
        s->setClientDisconnectedCallback(disconnectedCallback);
        s->setCallback(callback);
    }
    else 
        ROS_INFO("ERROR");

    //pub.publish(msg);
    ros::spin();

    return 0;
}
