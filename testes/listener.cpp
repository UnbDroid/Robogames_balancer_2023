// Includes ------------------------------------------------------------------------------------------------

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

// Defines ------------------------------------------------------------------------------------------------------------

//Cálculos
#define PI 3.14159265359f
#define RAIO_RODA 0.0575f // 5.75cm
#define PERIODO 30000 // TODO microssegundos
#define PUBLISH_RATE_HZ 70
#define QUEUE_SIZE 5000

// Controle de velocidade
#define POT_MAX_ESQUERDA 1
#define POT_MAX_DIREITA 1

#define POT_MIN_ESQUERDA 0.01f
#define POT_MIN_DIREITA 0.01f

//Pinagem
#define GPS_HEADER_PIN_PWM_DIREITA_3 2
#define GPS_HEADER_PIN_PWM_ESQUERDA_4 3

#define GPIO_PIN_1_17 17 
#define GPIO_PIN_1_25 25 

#define GPIO_PIN_3_17 17 
#define GPIO_PIN_3_20 20

// Variáveis ----------------------------------------------------------4-----------------------------------

// Encoders
int encoder0Pos = 0; 
int encoder1Pos = 0;
double voltas_esquerda = 0, voltas_esquerda_anterior = 0;
double voltas_direita = 0, voltas_direita_anterior = 0;

// Controlador
float velocidade_esquerda = 0;
float velocidade_direita = 0;

float velocidade_direita_old = 0;
float velocidade_esquerda_old = 0;

float erro_direita = 0;
float erro_esquerda = 0;

float velocidade_referencia = 0;
float velocidade_referencia_old = 0;

float somatorio_erro_direita = 0;
float somatorio_erro_esquerda = 0;

float potencia_motor_direita = 0;
float potencia_motor_esquerda = 0;


//Constantes do controlador
float Kp_esquerda = 0.3;
float Ki_esquerda = 0.000001;

float Kp_direita = 0.3;
float Ki_direita = 0.000001;

int saturadoDireito = 0;
int saturadoEsquerdo = 0;

//Cálculos
float comprimento_roda = 2 * PI * RAIO_RODA;


//Declração de funções -----------------------------------------------------------------------------------------------------

//Funções dos motores
void motorEsquerda(float potEsquerda)
{
    int praTras = 0;
    int parado = 0;
    saturadoEsquerdo = 0;

    if (potEsquerda < 0)
    {
        praTras = 1;
        potEsquerda = -potEsquerda;
    }

    if (potEsquerda >= POT_MAX_ESQUERDA)
    {
        potEsquerda = POT_MAX_ESQUERDA;
        saturadoEsquerdo = 1;
    }
    else if (potEsquerda <= POT_MIN_ESQUERDA)
    {
        parado = 1;
    }

    if (parado)
    {
        // Setar os pinos GPIO LOW e HIGH
        rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
        rc_gpio_set_value(1, GPIO_PIN_1_25, 0);
    }
    else if (praTras)
    {
        rc_pwm_set_duty(0, 'A', potEsquerda);

        rc_gpio_set_value(1, GPIO_PIN_1_25, 0);
        rc_gpio_set_value(1, GPIO_PIN_1_17, 1);
    }
    else
    {
        rc_pwm_set_duty(0, 'A', potEsquerda);

        rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
        rc_gpio_set_value(1, GPIO_PIN_1_25, 1);
    }
}

void motorDireita(float potDireita)
{

    int praTras = 0;
    int parado = 0;

    saturadoDireito = 0;

    if (potDireita < 0)
    {
        praTras = 1;
        potDireita = -potDireita;
    }

    if (potDireita >= POT_MAX_DIREITA)
    {
        potDireita = POT_MAX_DIREITA;
        saturadoDireito = 1;
    }
    else if (potDireita <= POT_MIN_DIREITA)
    {
        parado = 1;
    }

    if (parado)
    {
        // Setar os pinos GPIO LOW e HIGH
        rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
        rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
    }
    else if (praTras)
    {
        rc_pwm_set_duty(0, 'B', potDireita);
        rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
        rc_gpio_set_value(3, GPIO_PIN_3_17, 1);
    }
    else
    {
        rc_pwm_set_duty(0, 'B', potDireita);
        rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
        rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
    }
}

//Função do encoder
int encoder(int canal)
{
    int encoder = rc_encoder_eqep_read(canal);
    return encoder;
}


//Funções de cálculos

unsigned int micros()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t); // change CLOCK_MONOTONIC_RAW to CLOCK_MONOTONIC on non linux computers
    return t.tv_sec * 1000 + (t.tv_nsec + 500000) / 1000;
}

double voltasParaMetros(double voltas)
{
    return voltas * comprimento_roda;
}

float velocidadeParaMetros(float velocidade)
{
    return velocidade * comprimento_roda;
}


//Funções do ROS

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    float teste = msg->data;
    velocidade_referencia = teste;
}

// Código Principal ---------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{

    //Driver -------------------------------------------------------------------------------------------------------

    //Inicializa encoders
    if (rc_encoder_eqep_init())
    {
        fprintf(stderr, "ERROR: failed to run rc_encoder_eqep_init\n");
        return -1;
    }

    // Inicializa os pinos
    rc_gpio_init(1, GPIO_PIN_1_17, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(1, GPIO_PIN_1_25, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_17, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_20, GPIOHANDLE_REQUEST_OUTPUT);

    // Seta os pinos como pinos PWM
    rc_pinmux_set(GPS_HEADER_PIN_PWM_DIREITA_3, PINMUX_PWM);
    rc_pinmux_set(GPS_HEADER_PIN_PWM_ESQUERDA_4, PINMUX_PWM);

    // Inicializa PWM com 50 HZ
    rc_pwm_init(0, 50);

    //ROS ------------------------------------------------------------------------------

    //Inicializa e cria o nó
    ros::init(argc, argv, "controle_velocidade");
    ros::NodeHandle n;

    //Declaração de Publishers e Subscriber
    ros::Publisher velocidade = n.advertise<std_msgs::Float32>("velocidade", QUEUE_SIZE);
    ros::Publisher posicao = n.advertise<std_msgs::Float64>("posicao", QUEUE_SIZE);
    ros::Subscriber sub = n.subscribe("referencia", QUEUE_SIZE, chatterCallback);
    // ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    //Seta frequência de publicação e escuta
    ros::Rate rate(PUBLISH_RATE_HZ);

    while (ros::ok())
    {
        float multiplicacaoDireita = velocidade_referencia_old * velocidade_referencia;
        if (multiplicacaoDireita <= 0.0)
        {
            somatorio_erro_direita = 0;
            somatorio_erro_esquerda = 0;
        }

        velocidade_referencia_old = velocidade_referencia;

        // Inputs

        encoder0Pos = encoder(3);
        encoder1Pos = encoder(2);

        // printf("Encoder: %d, %d \n", encoder0Pos, encoder1Pos);

        voltas_esquerda_anterior = voltas_esquerda;
        voltas_direita_anterior = voltas_direita;

        voltas_esquerda = -encoder0Pos / (double)4096;
        voltas_direita = encoder1Pos / (double)4096;

        velocidade_esquerda_old = velocidade_esquerda;
        velocidade_direita_old = velocidade_direita;

        // printf("Voltas: %f, %f \n", voltas_direita, voltas_esquerda);

        velocidade_esquerda = 1000000 * (voltas_esquerda - voltas_esquerda_anterior) / ((double)(PERIODO));
        velocidade_direita = 1000000 * (voltas_direita - voltas_direita_anterior) / ((double)(PERIODO));

        if (!(velocidade_esquerda >= -2.5 && velocidade_esquerda <= 2.5))
        {
            velocidade_esquerda = velocidade_esquerda_old;
        }

        if (!(velocidade_direita >= -2.5 && velocidade_direita <= 2.5))
        {
            velocidade_direita = velocidade_direita_old;
        }

        // Controle
        erro_esquerda = velocidade_referencia - velocidade_esquerda;
        erro_direita = velocidade_referencia - velocidade_direita;

        if (saturadoEsquerdo == 0)
        {
            somatorio_erro_esquerda += erro_esquerda;
        }

        if (saturadoDireito == 0)
        {
            somatorio_erro_direita += erro_direita;
        }

        potencia_motor_esquerda = erro_esquerda * Kp_esquerda + (somatorio_erro_esquerda * Ki_esquerda) * PERIODO;
        potencia_motor_direita = erro_direita * Kp_direita + (somatorio_erro_direita * Ki_direita) * PERIODO;

        // Output
        motorEsquerda(potencia_motor_esquerda);
        motorDireita(potencia_motor_direita);

        //Plotar valores no gráfico
        // std_msgs::Float32MultiArray msg;
        // msg.data.push_back(velocidade_esquerda);
        // msg.data.push_back(somatorio_erro_esquerda);
        // msg.data.push_back(velocidade_referencia);
        // msg.data.push_back(erro_esquerda);

        static ros::Time last_publish_time = ros::Time::now();

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            std_msgs::Float64 posicaoMsg;
            double voltas_media = (voltas_direita + voltas_esquerda) / 2;
            double voltas_metros = voltasParaMetros(voltas_media);
            posicaoMsg.data = voltas_metros;
            posicao.publish(posicaoMsg);

            std_msgs::Float32 velocidadeMsg;
            float velocidade_media = (velocidade_direita + velocidade_esquerda) / 2;
            float velocidade_metros = velocidadeParaMetros(velocidade_media);
            velocidadeMsg.data = velocidade_metros;
            velocidade.publish(velocidadeMsg);
        }

        // printf("Potência Esquerda: %f, Velocidade esquerda: %f, encoder esquerda: %d, referência: %f \n", potencia_motor_esquerda, velocidade_esquerda, encoder0Pos, velocidade_referencia);
        // printf("Potência Direita: %f, Velocidade direita: %f, encoder direita: %d, referência: %f \n", potencia_motor_direita, velocidade_direita, encoder1Pos, velocidade_referencia);
        // printf("velocidade metros: %f, posicao metros: %f \n", velocidade_metros, voltas_metros);

        ros::spinOnce();
        rate.sleep();
    }

    rc_cleanup();
    rc_encoder_eqep_cleanup();

    return 0;
}
