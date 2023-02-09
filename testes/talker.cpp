#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include <termios.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("referencia", 10);

    while (ros::ok())
    {
        float input1;
        std::cin >> input1;

        std_msgs::Float32 msg;

        msg.data = input1;

        chatter_pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}