#include "ros/ros.h"
#include "std_msgs/Float32.h"

void imuCallback(const std_msgs::Float32::ConstPtr &msg)
{
    ROS_INFO("Escutando Angulo: [%lf]", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImuListener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("angle", 1000, imuCallback);

    ros::spin();

    return 0;
}