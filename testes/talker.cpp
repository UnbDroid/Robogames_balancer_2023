#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Int32.h"
#include <termios.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("chatter", 10);

    while (ros::ok())
    {
        int input;
        std::cin >> input;
        std_msgs::Int32 msg;
        msg.data = input;
        chatter_pub.publish(msg);
        ros::spinOnce();
    }

    return 0;
}