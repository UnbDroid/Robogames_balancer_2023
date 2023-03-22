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
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include <rc/time.h>
#include <inttypes.h>
#include <rc/led.h>

#define PERIODO 30000 // TODO microssegundos
#define MAX_ANGLE 0.2
#define PUBLISH_RATE_HZ 50

float error = 0;

float referenceTheta = 0;
float referenceVelocidade = 0;
float referencePosicao = 0;
float referenceOmega = 0;
float theta = 0;
float referencia = 0;
float velocidade = 0;
double posicao = 0;
float omega = 0;

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{

    theta = msg->data;
}

void velocidadeCallback(const std_msgs::Float32::ConstPtr &msg)
{

    velocidade = msg->data;
}

void posicaoCallback(const std_msgs::Float64::ConstPtr &msg)
{
    posicao = msg->data;
}

void angleThetaCallback(const std_msgs::Float32::ConstPtr &msg)
{

    omega = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "equilibrio");

    ros::NodeHandle n;

    ros::Publisher referencia_pub = n.advertise<std_msgs::Float32>("referencia", 10);

    // ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    ros::Subscriber angleSub = n.subscribe("angle", 10, chatterCallback);
    ros::Subscriber velocidadeSub = n.subscribe("velocidade", 10, velocidadeCallback);
    ros::Subscriber posicaoSub = n.subscribe("posicao", 10, posicaoCallback);
    ros::Subscriber angleThetaSub = n.subscribe("angle_theta_ponto", 10, angleThetaCallback);

    ros::Rate rate(PUBLISH_RATE_HZ);

    float K[4] = {38.375, 4.4658, 3.1623, 5.0738};

    float error[4];

    while (ros::ok())
    {
        error[0] = theta - referenceTheta;
        error[1] = omega - referenceOmega;
        error[2] = velocidade - referenceVelocidade;
        error[3] = posicao - referencePosicao;

        // Controlador

        referencia = 0;

        for (int i = 0; i < 4; i++)
        {
            referencia += error[i] * K[i];
        }

        if (theta < 0)
        {
            rc_led_set(RC_LED_BAT100, 1);
        }
        else
        {
            rc_led_set(RC_LED_BAT100, 0);
        }

        static ros::Time last_publish_time = ros::Time::now();

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            // REFERENCIA
            std_msgs::Float32 msgReferencia;
            msgReferencia.data = referencia;
            referencia_pub.publish(msgReferencia);
        }

        printf("%f, %f\n", theta, referencia);

        ros::spinOnce();
        rate.sleep();
    }
}