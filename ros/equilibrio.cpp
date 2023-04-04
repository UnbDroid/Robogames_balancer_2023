
//Includes ------------------------------------------------------------------------------------------------------------
#include <math.h>
#include <robotcontrol.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include <rc/led.h>


//Defines ------------------------------------------------------------------------------------------------------------------
#define PUBLISH_RATE_HZ 70
#define QUEUE_SIZE 5000

//Variáveis --------------------------------------------------------------------------------------------------------------------

float referenceTheta = 0;
float referenceVelocidade = 0;
float referencePosicao = 0;
float referenceOmega = 0;

float theta = 0;
float referencia = 0;
float velocidade = 0;
double posicao = 0;
float omega = 0;


//Funções do ROS
void thetaCallback(const std_msgs::Float32::ConstPtr &msg)
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

void omegaCallback(const std_msgs::Float32::ConstPtr &msg)
{
    omega = msg->data;
}

void referenciaPosicaoCallback(const std_msgs::Float32::ConstPtr &msg)
{
    referencePosicao = msg->data;
}

void referenciaVelocidadeCallback(const std_msgs::Float32::ConstPtr &msg)
{
    referenceVelocidade = msg->data;
}

//Código Principal ---------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{

    //Inicializa e cria o nó
    ros::init(argc, argv, "equilibrio");
    ros::NodeHandle n;

    //Declaração dos Publishers
    ros::Publisher referencia_pub = n.advertise<std_msgs::Float32>("referencia", 50);
    // ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    //Declaração dos Subscribes
    ros::Subscriber angleSub = n.subscribe("angle", QUEUE_SIZE, thetaCallback);
    ros::Subscriber velocidadeSub = n.subscribe("velocidade", QUEUE_SIZE, velocidadeCallback);
    ros::Subscriber posicaoSub = n.subscribe("posicao", QUEUE_SIZE, posicaoCallback);
    ros::Subscriber angleThetaSub = n.subscribe("angle_theta_ponto", QUEUE_SIZE, omegaCallback);
    ros::Subscriber referenciaPosicao = n.subscribe("referencia_posicao", QUEUE_SIZE, referenciaPosicaoCallback);
    ros::Subscriber referenciaVelocidade = n.subscribe("referencia_velocidade", QUEUE_SIZE, referenciaVelocidadeCallback);

    //Seta frequência de publicação e escuta
    ros::Rate rate(PUBLISH_RATE_HZ);

    //Controlador Variáveis e Constantes -----------------------------------------------------------------------------------------------------
    float K[4] = {55.2761, 5.3796, 3.1623, 5.5528};
    float error[4];

    while (ros::ok())
    {
        error[0] = theta - referenceTheta;
        error[1] = omega - referenceOmega;
        error[2] = posicao - referencePosicao;
        error[3] = velocidade - referenceVelocidade;

        // Controlador
        referencia = 0;

        for (int i = 0; i < 4; i++)
        {
            referencia += error[i] * K[i];
        }

        //Acende o LED toda a vez que troca de lado
        if (theta < 0)
        {
            rc_led_set(RC_LED_BAT100, 1);
        }
        else
        {
            rc_led_set(RC_LED_BAT100, 0);
        }


        //Envia a referência para o controlador de velocidade
        static ros::Time last_publish_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            // REFERENCIA
            std_msgs::Float32 msgReferencia;
            msgReferencia.data = referencia;
            referencia_pub.publish(msgReferencia);
        }

        //Plota gráficos
        std_msgs::Float32MultiArray msg;
        // msg.data.push_back(theta);
        // msg.data.push_back(omega);
        // msg.data.push_back(referencia);

        // msg.data.push_back(velocidade);
        // msg.data.push_back(posicao);
        // msg.data.push_back(referenceVelocidade);
        // msg.data.push_back(referencePosicao);

        float referencia0 =  error[0] * K[0];
        float referencia1 =  error[1] * K[1];
        float referencia2 =  error[2] * K[2];
        float referencia3 =  error[3] * K[3];

        // msg.data.push_back(referenceVelocidade);
        // msg.data.push_back(referencePosicao);
        // msg.data.push_back(referencia0);
        // msg.data.push_back(referencia1);
        // msg.data.push_back(referencia2);
        // msg.data.push_back(referencia3);
        // chatter_pub.publish(msg);

        // printf("%f, %f, %f, %f, %f, %f \n", referenceVelocidade, referencePosicao, referencia0, referencia1, referencia2, referencia3);


        ros::spinOnce();
        rate.sleep();
    }
}