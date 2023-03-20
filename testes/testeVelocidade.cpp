#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include <termios.h>

#define PUBLISH_RATE_HZ 80

float v = 1.0;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("referencia", 10);
    ros::Rate rate(PUBLISH_RATE_HZ);

    while (ros::ok())
    {

        static ros::Time last_publish_time = ros::Time::now();

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            float input1 = (current_time - last_publish_time).toSec();

            std_msgs::Float32 msg;

            msg.data = input1;

            chatter_pub.publish(msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}