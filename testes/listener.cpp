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

// Calculo de velocidade
#define pi 3.14159265359f

// Controle de velocidade
#define limiar_erro_velocidade 0.07f
#define POT_MAX_ESQUERDA 1
#define POT_MAX_DIREITA 1
#define POT_MIN_ESQUERDA 0.01f
#define POT_MIN_DIREITA 0.01f
// #define Kp_esquerda 2.0f
// #define Ki_esquerda 0.2f
// #define Kp_direita 1.5f
// #define Ki_direita 0.18f

#define GPS_HEADER_PIN_PWM_DIREITA_3 2
#define GPS_HEADER_PIN_PWM_ESQUERDA_4 3
#define GPIO_PIN_1_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_1_25 25 // Set the GPIO pin for the motor control

#define GPIO_PIN_3_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_3_20 20

#define PERIODO 30000 // TODO microssegundos

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

float somatorio_erro_direita = 0;
float somatorio_erro_esquerda = 0;

float potencia_motor_direita = 0;
float potencia_motor_esquerda = 0;

int running;

float Kp_esquerda = 1.0;
float Ki_esquerda = 0.0;
float Kp_direita = 0.3;
float Ki_direita = 0.000001;

int saturadoDireito = 0;
int saturadoEsquerdo = 0;

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

        rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
        rc_gpio_set_value(1, GPIO_PIN_1_25, 0); // 1
    }
    else
    {
        rc_pwm_set_duty(0, 'A', potEsquerda);

        rc_gpio_set_value(1, GPIO_PIN_1_25, 0);
        rc_gpio_set_value(1, GPIO_PIN_1_17, 0); // 1
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
        rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
        rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
    }
    else
    {
        rc_pwm_set_duty(0, 'B', potDireita);
        rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
        rc_gpio_set_value(3, GPIO_PIN_3_17, 1);
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

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    Kp_direita = msg->data;
    ROS_INFO("Escutei KP: %f", msg->data);
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

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("chatter", 10);

    int i = 0;

    // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    while (running)
    {
        float multiplicacao = velocidade_referencia_old * velocidade_referencia;
        if (multiplicacao <= 0.0){
            somatorio_erro_direita = 0;
            somatorio_erro_esquerda = 0;
            printf("Entrou no if: %f, %f\n", somatorio_erro_direita, somatorio_erro_esquerda);
        }

            velocidade_referencia_old = velocidade_referencia;

        tempo = micros();

        if ((i++) * PERIODO < 1000000)
        {
            velocidade_referencia = 1.5;
            rc_led_set(RC_LED_BAT25, 1);
        }
        else if (i * PERIODO < 2000000)
        {
            velocidade_referencia = -1.5;
            rc_led_set(RC_LED_BAT25, 0);
        }
        else
        {
            i = 0;
        }

        // Inputs

        encoder0Pos = encoder(2);
        encoder1Pos = encoder(3);

        voltas_esquerda = -encoder0Pos / (double)4096;
        voltas_direita = encoder1Pos / (double)4096;

        velocidade_esquerda = 1000000 * (voltas_esquerda - voltas_esquerda_anterior) / ((double)(PERIODO));
        velocidade_direita = 1000000 * (voltas_direita - voltas_direita_anterior) / ((double)(PERIODO));

        voltas_esquerda_anterior = voltas_esquerda;
        voltas_direita_anterior = voltas_direita;

        // Controle
        erro_direita = velocidade_referencia - velocidade_direita;
        erro_esquerda = velocidade_referencia - velocidade_esquerda;

        if (saturadoDireito == 0)
        {
            somatorio_erro_direita += erro_direita;
        }

        if (saturadoEsquerdo == 0)
        {
            somatorio_erro_esquerda += erro_esquerda;
        }

        potencia_motor_direita = erro_direita * Kp_direita + (somatorio_erro_direita * Ki_direita) * PERIODO;
        potencia_motor_esquerda = erro_esquerda * Kp_esquerda + (somatorio_erro_esquerda * Ki_esquerda) * PERIODO;

        // Output
        motorDireita(potencia_motor_direita);
        motorEsquerda(potencia_motor_esquerda);

        printf("%f, %f, %f \n", velocidade_direita, somatorio_erro_direita, somatorio_erro_esquerda);

        std_msgs::Float32 msg;
        msg.data = velocidade_direita;
        if (velocidade_direita > -3.0 && velocidade_direita < 3.0)
        {
            chatter_pub.publish(msg);
        }
        ROS_INFO("%f", velocidade_direita);
        ros::spinOnce();

        // Lidando com período
        rc_usleep(PERIODO - (micros() - tempo));
    }

    rc_cleanup();
    rc_encoder_eqep_cleanup();

    return 0;
}
