#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "custom_messages/Axis.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "joystick_receiver");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<custom_messages::Axis>("joystick_cmd", 1000);
    ros::Rate loop_rate(3);

    custom_messages::Axis msg;
    msg.pitch=0;
    msg.roll=0;
    msg.throtle=0;
    msg.yaw=0;

    while(ros::ok()){   

        pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}