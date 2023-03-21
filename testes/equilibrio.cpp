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
#define MAX_ANGLE 0.2

float error = 0;

float referenceAngle = 0;
float measurement = 0;
float referencia = 0;
int tempo = 0;
float derivative = 0;
float previous_error = 0;


uint64_t ultimoCiclo = 0;

float somatorio_error = 0;

int running;

// float Kp = 200;
// float Ki = 0.01; // 0.001 - 0.01

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

    ros::Publisher referencia_pub = n.advertise<std_msgs::Float32>("referencia", 10);

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);


    ros::Subscriber sub = n.subscribe("angle", 10, chatterCallback);

    ros::Rate rate(80);

    float Kp = 0;
    n.param<float>("Kp", Kp, 0.1f);

    float Ki = 0;
    n.param<float>("Ki", Ki, 0.0f);

    float Kd = 0;
    n.param<float>("Kd", Kd, 0.0f);

    // ultimoCiclo = rc_nanos_since_boot()/1000;

    printf("Equilibrio Started");

    while (ros::ok())
    {

        ros::spinOnce();

        // Controle
        error = referenceAngle - measurement;
        somatorio_error += error;

        derivative = error - previous_error;
        previous_error = error;

        referencia = error * Kp + (somatorio_error * Ki) + (derivative * 0);

        // printf("%f, %f, %f \n", measurement, error, referencia);


        //REFERENCIA
        std_msgs::Float32 msgReferencia;
        msgReferencia.data = referencia;
        referencia_pub.publish(msgReferencia);
        

        //TESTE DE PLOT
        // std_msgs::Float32MultiArray msg;
        // msg.data.push_back(measurement);
        // msg.data.push_back(referencia/100000);
        // chatter_pub.publish(msg);

        printf("%f, %f\n", measurement, referencia);

        rate.sleep();

    }
}