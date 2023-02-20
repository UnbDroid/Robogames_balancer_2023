#include <math.h>
#include <robotcontrol.h>
#include <stdio.h>
#include <linux/gpio.h>
#include <rc/pwm.h>
#include <rc/pinmux.h>
#include <rc/encoder_eqep.h>
#include <time.h>
#include <signal.h>
#include <termios.h>
#include <sys/time.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <rc/time.h>
#include <inttypes.h>

#define PERIODO 30000 // TODO microssegundos

float error = 0;

float setpoint = 0;
float measurement = 0;
float referencia = 0;
int tempo = 0;
uint64_t ultimoCiclo = 0;

float somatorio_error = 0;

int running;

float Kp = 0.3;
float Ki = 0.000001;

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

unsigned long int micros()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t); // change CLOCK_MONOTONIC_RAW to CLOCK_MONOTONIC on non linux computers
    return t.tv_sec * 1000 + (t.tv_nsec + 500000) / 1000;
}

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    measurement = msg->data;
}

int main(int argc, char *argv[])
{

    signal(SIGINT, __signal_handler);
    running = 1;

    ros::init(argc, argv, "equilibrio");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("referencia", 10);

    ros::Subscriber sub = n.subscribe("angle", 10, chatterCallback);

    ros::Rate rate(50);

    // ultimoCiclo = rc_nanos_since_boot()/1000;

    while (ros::ok())
    {

        // Controle
        error = setpoint - measurement;
        somatorio_error += error;

        referencia = error * Kp + (somatorio_error * Ki);

        // printf("%f, %f, %f \n", measurement, error, referencia);

        std_msgs::Float32 msg;
        msg.data = referencia;

        chatter_pub.publish(msg);
        ROS_INFO("%f", msg.data);

        // uint64_t agora = rc_nanos_since_boot();

        // if(agora - ultimoCiclo > PERIODO){
        //     printf("Ciclo dura mais que período: agora(%lu) - ultimoCiclo(%lu) = %ld > PERIODO(%lu)\n",
        //     agora, ultimoCiclo, agora-ultimoCiclo, PERIODO);

        // }

        // if(ultimoCiclo > agora){
        //     printf("Último ciclo maior que agora: ultimoCiclo(%lu) > agora(%lu) \n",
        //     agora, ultimoCiclo, agora-ultimoCiclo, PERIODO);
        // }

        // while(agora - ultimoCiclo < PERIODO)
        // {
        //     rc_usleep(1);
        //     agora = rc_nanos_since_boot();
        // }

        // ultimoCiclo = agora;
        ros::spinOnce();
        rate.sleep();
        // printf("agora:%u ns\n", agora);
    }
}