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

#define PERIODO 30000 // TODO microssegundos

float error = 0;

float setpoint = 0;
float measurement = 0;
float referencia = 0;
int tempo = 0;


float somatorio_error = 0;

int running;

float Kp = 0.3;
float Ki = 0.000001;


static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

unsigned int micros()
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

    ros::Subscriber sub = n.subscribe("angle", 1000, chatterCallback);

    while (running)
    {

        tempo = micros();

        // Controle
        error = setpoint - measurement;
        somatorio_error += error;

        referencia = error * Kp + (somatorio_error * Ki);

        // printf("%f, %f, %f \n", measurement, error, referencia);

        std_msgs::Float32 msg;
        msg.data = referencia;

        chatter_pub.publish(msg);
        ros::spinOnce();

        // Lidando com período
        int testePeriodo = PERIODO - (micros() - tempo);

        if (testePeriodo > 0)
        {
            // Lidando com período
            rc_usleep(testePeriodo);
        }
        else
        {
            perror("DEU RUIM RAPAZ!! PROCESSAAAAMENTO ...\n");
        }
    }
}