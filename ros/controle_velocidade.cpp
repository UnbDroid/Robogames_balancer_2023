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

// Cálculos
#define PI 3.14159265359f
#define RAIO_RODA 0.0575f // 5.75cm

// Controle de velocidade
#define POT_MAX_ESQUERDA 1
#define POT_MAX_DIREITA 1

#define POT_MIN_ESQUERDA 0.01f
#define POT_MIN_DIREITA 0.01f

// Pinagem
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

// Constantes do controlador
float Kp_esquerda = 0.5; // 0.3
float Ki_esquerda = 0;

float Kp_direita = 0.5;
float Ki_direita = 0; // 0.000001

int saturadoDireito = 0;
int saturadoEsquerdo = 0;

float theta = 0;
float referencia = 0;
float velocidade = 0;
double posicao = 0;
float omega = 0;

float referenceTheta = 0;
float referenceVelocidade = 0;
float referencePosicao = 0;
float referenceOmega = 0;

// Cálculos
float comprimento_roda = 2 * PI * RAIO_RODA;

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
#define SAMPLE_RATE_HZ 50
#define PUBLISH_RATE_HZ 5000
#define QUEUE_SIZE 5000
#define MEDIA_MODEL_SIZE 50
#define TB_ROLL_Y 1
#define NUM_PARAMS 4
#define MAX_ERRO_POS 0.3f

static rc_mpu_data_t mpu_data;

static float prev_angle = 0.0;
static ros::Time prev_time;

// Declração de funções -----------------------------------------------------------------------------------------------------

// Funções dos motores
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

// Função do encoder
int encoder(int canal)
{
    int encoder = rc_encoder_eqep_read(canal);
    return encoder;
}

// Funções de cálculos

double voltasParaMetros(double voltas)
{
    return voltas * comprimento_roda;
}

float velocidadeParaMetros(float velocidade)
{
    return velocidade * comprimento_roda;
}

// Funções do ROS

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    float teste = msg->data;
    velocidade_referencia = teste;
}

// Código Principal ---------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{

    // Driver -------------------------------------------------------------------------------------------------------

    // Inicializa encoders
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

    // ROS ------------------------------------------------------------------------------

    // Inicializa e cria o nó
    ros::init(argc, argv, "controle_velocidade");
    ros::NodeHandle n;

    // Declaração de Publishers e Subscriber
    ros::Publisher velocidade = n.advertise<std_msgs::Float32>("velocidade", QUEUE_SIZE);
    ros::Publisher posicao = n.advertise<std_msgs::Float64>("posicao", QUEUE_SIZE);

    ros::Subscriber sub = n.subscribe("referencia", QUEUE_SIZE, chatterCallback);
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    // Seta frequência de publicação e escuta
    ros::Rate rate(PUBLISH_RATE_HZ);

    static ros::Time last_publish_time = ros::Time::now();

    float velocidade_esquerda[MEDIA_MODEL_SIZE] = {0};
    float velocidade_direita[MEDIA_MODEL_SIZE] = {0};

    int count_velocidade = 0;

    float velocidade_esquerda_media = 0;
    float velocidade_direita_media = 0;


    while (ros::ok())
    {

        ros::Time current_time = ros::Time::now();

        ros::Duration dt = current_time - last_publish_time;

        if ((dt).toSec() >= 1.0 / PUBLISH_RATE_HZ)
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

            // velocidade_esquerda_old = velocidade_esquerda;
            // velocidade_direita_old = velocidade_direita;

            velocidade_esquerda[count_velocidade] = (voltasParaMetros(voltas_esquerda - voltas_esquerda_anterior)) / dt.toSec();
            velocidade_direita[count_velocidade] = (voltasParaMetros(voltas_direita - voltas_direita_anterior)) / dt.toSec();

            velocidade_direita_media += velocidade_direita[count_velocidade] / MEDIA_MODEL_SIZE;
            velocidade_esquerda_media += velocidade_esquerda[count_velocidade] / MEDIA_MODEL_SIZE;

            count_velocidade = (count_velocidade + 1) % MEDIA_MODEL_SIZE;

            velocidade_direita_media -= velocidade_direita[count_velocidade] / MEDIA_MODEL_SIZE;
            velocidade_esquerda_media -= velocidade_esquerda[count_velocidade] / MEDIA_MODEL_SIZE;

            // if (!(velocidade_esquerda >= -2.5 && velocidade_esquerda <= 2.5))
            // {
            //     velocidade_esquerda = velocidade_esquerda_old;
            // }

            // if (!(velocidade_direita >= -2.5 && velocidade_direita <= 2.5))
            // {
            //     velocidade_direita = velocidade_direita_old;
            // }

            double voltas_media = (voltas_direita + voltas_esquerda) / 2;
            double voltas_metros = voltasParaMetros(voltas_media);
            posicao = voltas_metros;

            float velocidade_media = (velocidade_direita_media + velocidade_esquerda_media) / 2;
            velocidade = velocidade_media;


            // Controle
            erro_esquerda = velocidade_referencia - velocidade_esquerda_media;
            erro_direita = velocidade_referencia - velocidade_direita_media;

            if (saturadoEsquerdo == 0)
            {
                somatorio_erro_esquerda += erro_esquerda;
            }

            if (saturadoDireito == 0)
            {
                somatorio_erro_direita += erro_direita;
            }

            potencia_motor_esquerda = erro_esquerda * Kp_esquerda + (somatorio_erro_esquerda * Ki_esquerda) * dt.toSec();
            potencia_motor_direita = erro_direita * Kp_direita + (somatorio_erro_direita * Ki_direita) * dt.toSec();

            // Output
            motorEsquerda(potencia_motor_esquerda);
            motorDireita(potencia_motor_direita);


        }
        // rc_led_set(RC_LED_BAT100, 0);
        last_publish_time = current_time;

        rate.sleep();
        ros::spinOnce();
    }

    rc_cleanup();
    rc_encoder_eqep_cleanup();

    rc_mpu_set_dmp_callback(NULL);
    rc_mpu_power_off();

    return 0;
}
