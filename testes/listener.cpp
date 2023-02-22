// Includes ------------------------------------------------------------------------------------------------

// Cálculos
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

// Calculo de velocidade
#define pi 3.14159265359f

// Controle de velocidade
#define limiar_erro_velocidade 0.07f
#define POT_MAX_ESQUERDA 1
#define POT_MAX_DIREITA 1
#define POT_MIN_ESQUERDA 0.01f
#define POT_MIN_DIREITA 0.01f

#define GPS_HEADER_PIN_PWM_DIREITA_3 2
#define GPS_HEADER_PIN_PWM_ESQUERDA_4 3
#define GPIO_PIN_1_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_1_25 25 // Set the GPIO pin for the motor control

#define GPIO_PIN_3_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_3_20 20

#define PERIODO 30000 // TODO microssegundos
#define PUBLISH_RATE_HZ 5

// Variáveis ----------------------------------------------------------4-----------------------------------

// Encoders
int encoder0Pos = 0; // esquerda
int encoder1Pos = 0; // direita
double voltas_esquerda = 0, voltas_esquerda_anterior = 0;
double voltas_direita = 0, voltas_direita_anterior = 0;

// Cálculo de velocidade
float velocidade_esquerda = 0;
float velocidade_direita = 0;
int tempo = 0;
int periodo = 0;
int tempoOld = 0;

// Driver
float pot_direita = 0;
float pot_esquerda = 0;
float velocidade_Referencia;

float erro_direita = 0;
float erro_esquerda = 0;

float velocidade_referencia = 0;
float velocidade_referencia_old = 0;

float velocidade_referencia_esquerda = 1.5;
float velocidade_referencia_esquerda_old = 0;

float somatorio_erro_direita = 0;
float somatorio_erro_esquerda = 0;

float potencia_motor_direita = 0;
float potencia_motor_esquerda = 0;

int running;

float Kp_esquerda = 0.3;
float Ki_esquerda = 0.000001;
float Kp_direita = 0.3;
float Ki_direita = 0.000001;

int saturadoDireito = 0;
int saturadoEsquerdo = 0;

float velocidade_direita_old = 0;
float velocidade_esquerda_old = 0;

float taxa_desaceleracao = 0.1; // Ajuste de acordo com a necessidade
float velocidade_maxima = 3.0;  // Ajuste de acordo com a necessidade
float velocidade_referencia_desejada = 0.0;

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

int encoder(int canal)
{
    int encoder = rc_encoder_eqep_read(canal);
    return encoder;
}

void controleAdaptativoVelocidade()
{
}

// Setup e Loop principais
//-----------------------------------------------------------------------------------------------------------

unsigned int micros()
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t); // change CLOCK_MONOTONIC_RAW to CLOCK_MONOTONIC on non linux computers
    return t.tv_sec * 1000 + (t.tv_nsec + 500000) / 1000;
}

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

// float atualizar_velocidade_referencia(float velocidade_referencia_atual, float velocidade_referencia_desejada, float taxa_desaceleracao, float velocidade_maxima) {
//   if (velocidade_referencia_atual > velocidade_referencia_desejada) {
//     velocidade_referencia_desejada += taxa_desaceleracao;
//     if (velocidade_referencia_desejada < 0.0) {
//       velocidade_referencia_desejada = 0.0;
//     }
//   } else if (velocidade_referencia_atual < velocidade_referencia_desejada) {
//     velocidade_referencia_desejada -= taxa_desaceleracao;
//     if (velocidade_referencia_desejada < 0.0) {
//       velocidade_referencia_desejada = 0.0;
//     }
//   }
//   if (velocidade_referencia_desejada > velocidade_maxima) {
//     velocidade_referencia_desejada = velocidade_maxima;
//   }
//   return velocidade_referencia_desejada;
// }

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    static ros::Time last_publish_time = ros::Time::now();

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
    {
        float teste = msg->data;
        // velocidade_referencia = teste;
        if (teste > -1.0 && teste < 1.0)
        {
            velocidade_referencia = 0;
            // ROS_INFO("Escutei velocidade referencia entre -1.0 e 1.0: %f", velocidade_referencia);
        }
        else
        {
            velocidade_referencia = teste;
            // velocidade_referencia_desejada = atualizar_velocidade_referencia(velocidade_referencia, velocidade_referencia_desejada, taxa_desaceleracao, velocidade_maxima);
            // ROS_INFO("Escutei velocidade referencia: %f", velocidade_referencia);
        }
    }
}

int main(int argc, char *argv[])
{

    if (rc_encoder_eqep_init())
    {
        fprintf(stderr, "ERROR: failed to run rc_encoder_eqep_init\n");
        return -1;
    }

    // Inicialize GPIO pin for motor control
    rc_gpio_init(1, GPIO_PIN_1_17, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(1, GPIO_PIN_1_25, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_17, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_20, GPIOHANDLE_REQUEST_OUTPUT);

    // Set PWM pin
    rc_pinmux_set(GPS_HEADER_PIN_PWM_DIREITA_3, PINMUX_PWM);
    rc_pinmux_set(GPS_HEADER_PIN_PWM_ESQUERDA_4, PINMUX_PWM);

    // PWM with 50HZ
    rc_pwm_init(0, 50);

    signal(SIGINT, __signal_handler);
    running = 1;

    ros::init(argc, argv, "controle_velocidade");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    ros::Subscriber sub = n.subscribe("referencia", 1000, chatterCallback);

    ros::Rate rate(PUBLISH_RATE_HZ);

    int i = 0;

    while (ros::ok())
    {
        float multiplicacaoDireita = velocidade_referencia_old * velocidade_referencia;
        if (multiplicacaoDireita <= 0.0)
        {
            somatorio_erro_direita = 0;
            somatorio_erro_esquerda = 0;
        }

        velocidade_referencia_old = velocidade_referencia;

        // float multiplicacaoEsquerda = velocidade_referencia_esquerda_old * velocidade_referencia_esquerda;
        // if (multiplicacaoEsquerda <= 0.0)
        // {
        //     somatorio_erro_esquerda = 0;
        // }

        // velocidade_referencia_esquerda_old = velocidade_referencia_esquerda;

        // tempo = micros();

        // if ((i++) * PERIODO < 1000000)
        // {
        //     velocidade_referencia = 1.5;
        //     rc_led_set(RC_LED_BAT25, 1);
        // }
        // else if (i * PERIODO < 2000000)
        // {
        //     velocidade_referencia = -1.5;
        //     rc_led_set(RC_LED_BAT25, 0);
        // }
        // else
        // {
        //     i = 0;
        // }

        // Inputs

        encoder0Pos = encoder(3);
        encoder1Pos = encoder(2);

        printf("Encoder: %d, %d \n", encoder0Pos, encoder1Pos);

        voltas_esquerda = -encoder0Pos / (double)4096;
        voltas_direita = -encoder1Pos / (double)4096;

        printf("Voltas: %f, %f \n", voltas_direita, voltas_esquerda);

        velocidade_esquerda = 1000000 * (voltas_esquerda - voltas_esquerda_anterior) / ((double)(PERIODO));
        velocidade_direita = 1000000 * (voltas_direita - voltas_direita_anterior) / ((double)(PERIODO));

        if (velocidade_esquerda >= -4.0 && velocidade_esquerda <= 4.0)
        {
            velocidade_esquerda_old = velocidade_esquerda;
        }
        else
        {
            velocidade_esquerda = velocidade_esquerda_old;
        }

        if (velocidade_direita >= -4.0 && velocidade_direita <= 4.0)
        {
            velocidade_direita_old = velocidade_direita;
        }
        else
        {
            velocidade_direita = velocidade_direita_old;
        }

        voltas_esquerda_anterior = voltas_esquerda;
        voltas_direita_anterior = voltas_direita;

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

        // printf("Esquerda: %f, %f, %f, \n", velocidade_esquerda, somatorio_erro_esquerda, velocidade_referencia);
        // printf("Direita: %f, %f, %f, \n", velocidade_direita, somatorio_erro_direita, velocidade_referencia);

        // std_msgs::Float32MultiArray msg;
        // msg.data.push_back(velocidade_esquerda);
        // msg.data.push_back(somatorio_erro_esquerda);
        // msg.data.push_back(velocidade_referencia);
        // msg.data.push_back(erro_esquerda);

        // if (velocidade_direita > -3.0 && velocidade_direita < 3.0)
        // {
        //     chatter_pub.publish(msg);
        // }

        ros::spinOnce();
        rate.sleep();

        // int testePeriodo = PERIODO - (micros() - tempo);
        // printf("periodo: %d\n", testePeriodo);

        // if (testePeriodo > 0)
        // {
        //     // Lidando com período
        //     rc_usleep(testePeriodo);
        // }
        // else
        // {
        //     perror("DEU RUIM RAPAZ!! PROCESSAAAAMENTO ...\n");
        // }
    }

    rc_cleanup();
    rc_encoder_eqep_cleanup();

    return 0;
}
