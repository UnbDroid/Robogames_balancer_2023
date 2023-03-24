#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float32.h"
#include <termios.h>

#define PUBLISH_RATE_HZ 80

float posicao_referencia = 0.7;
float velocidade_referencia = 0.7;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("referencia_posicao", 10);
    ros::Publisher pub = n.advertise<std_msgs::Float32>("referencia_velocidade", 10);
    ros::Rate rate(PUBLISH_RATE_HZ);

    while (ros::ok())
    {

        static ros::Time last_publish_time = ros::Time::now();

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            float input1 = (current_time - last_publish_time).toSec()*posicao_referencia;

            std_msgs::Float32 msg, velocidade;
            msg.data = input1;
            velocidade.data = velocidade_referencia;
            chatter_pub.publish(msg);
            pub.publish(velocidade);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}