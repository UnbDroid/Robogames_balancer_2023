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
#include "Motor.h"

#include "ros/ros.h"
#include "std_msgs/Int32.h"

// SD card
#define CS 53

// Encoders esquerda
#define encoder0PinA 18
#define encoder0PinB 19

// Encoders direita
#define encoder1PinA 2
#define encoder1PinB 3

// Calculo de velocidade
#define pi 3.14159265359f

// Controle de velocidade
#define limiar_erro_velocidade 0.07f
#define POT_MAX_ESQUERDA 1
#define POT_MAX_DIREITA 1
#define POT_MIN_ESQUERDA 0.1f
#define POT_MIN_DIREITA 0.1f
#define Kp_esquerda 6.5f
#define Ki_esquerda 0.05f
#define Kd_esquerda 0.1f
#define Kp_direita 6.5f
#define Ki_direita 0.05f
#define Kd_direita 0.1f

// Controle remoto
#define CHANEL1 22
#define CHANEL2 24
#define CHANEL3 26

// Driver
#define IN1_D 10
#define IN2_D 9
#define IN1_E 13
#define IN2_E 12

#define PWM_D 8
#define PWM_E 11


#define GPS_HEADER_PIN_PWM_DIREITA_3 2
#define GPS_HEADER_PIN_PWM_ESQUERDA_4 3
#define GPIO_PIN_1_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_1_25 25 // Set the GPIO pin for the motor control

#define GPIO_PIN_3_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_3_20 20


// Variáveis ----------------------------------------------------------4-----------------------------------

// SD card
const int chipSelect = CS;
int dataString = 0;
bool abriu = true;
char nomearquivo[] = "datalog1.txt";

// Encoders
volatile long encoder0Pos = 0; // esquerda
volatile long encoder1Pos = 0; // direita
double voltas_esquerda = 0, voltas_esquerda_anterior = 0;
double voltas_direita = 0, voltas_direita_anterior = 0;

// Cálculo de velocidade
float velocidade_esquerda = 0;
float velocidade_direita = 0;
double tempo;
double tempo_aux;

// Controle de velocidade
float velocidade_Referencia = 1;
float velocidade_Referencia_anterior = 0;
bool trocar_sentido = false;
bool foward = true;
bool back = false;
bool left = false;
bool right = false;
bool left_eixo = false;
bool right_eixo = false;

float theta1_esquerda = 0.50;
float theta2_esquerda = 0.25;
float yTv_esquerda = 0.08;

float theta1_direita = 0.40;
float theta2_direita = 0.15;
float yTv_direita = 0.07;

float velocidade_esquerda_modelo = 0;
float velocidade_direita_modelo = 0;
float pot_esquerda_teste = 0;
float pot_direita_teste = 0;
float tensaomotor_esquerda = 0;
float tensaomotor_direita = 0;
float tensao_bateria = 15;

// Driver
float pot_direita = 0;
float pot_esquerda = 0;

// Ler int do buffer serial
float valor = 0;

// Usar o controle remoto
int ch1;
int ch2;
int ch3;

int running;

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

void frente(int potEsquerda, int potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 1);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 0);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
}

int encoder(int canal)
{
    int encoder = rc_encoder_eqep_read(canal);
    printf("canal: %d, encoder: %d\n", canal, encoder);
    return encoder;
}

void controleAdaptativoVelocidade()
{

    if (abs(velocidade_Referencia) <= 0.0001)
    {
        printf("ENTROU IF DE PARAR \n");
        // parar();
        return;
    }

    theta1_esquerda = theta1_esquerda - (yTv_esquerda * velocidade_Referencia * (velocidade_esquerda - velocidade_esquerda_modelo));
    theta2_esquerda = theta2_esquerda + (yTv_esquerda * velocidade_esquerda * (velocidade_esquerda - velocidade_esquerda_modelo));

    theta1_direita = theta1_direita - (yTv_direita * velocidade_Referencia * (velocidade_direita - velocidade_direita_modelo));
    theta2_direita = theta2_direita + (yTv_direita * velocidade_direita * (velocidade_direita - velocidade_direita_modelo));

    tensaomotor_esquerda = (theta1_esquerda * velocidade_Referencia) - (theta2_esquerda * velocidade_esquerda);

    tensaomotor_direita = (theta1_direita * velocidade_Referencia) - (theta2_direita * velocidade_direita);

    velocidade_esquerda_modelo = (0.00248 * velocidade_esquerda_modelo) + (0.99752 * velocidade_Referencia_anterior);

    velocidade_direita_modelo = (0.00248 * velocidade_direita_modelo) + (0.99752 * velocidade_Referencia_anterior);

    velocidade_Referencia_anterior = velocidade_Referencia;

    pot_esquerda = tensaomotor_esquerda / (tensao_bateria);
    pot_esquerda_teste = pot_esquerda;

    pot_direita = tensaomotor_direita / (tensao_bateria);
    pot_direita_teste = pot_direita;

    printf("Potência esquerda antes limitador: %f\n", pot_esquerda);

    if (abs(pot_esquerda) > POT_MAX_ESQUERDA)
    {
        pot_esquerda = POT_MAX_ESQUERDA;
    }
    else if (abs(pot_esquerda) < POT_MIN_ESQUERDA)
    {
        pot_esquerda = POT_MIN_ESQUERDA;
    }
    if (abs(pot_direita) > POT_MAX_DIREITA)
    {
        pot_direita = POT_MAX_DIREITA;
    }
    else if (abs(pot_direita) < POT_MIN_DIREITA)
    {
        pot_direita = POT_MIN_DIREITA;
    }

    if (foward)
    {
        frente(pot_esquerda, pot_direita);
    }

    printf("Potência controlada esquerda: %f\n", pot_esquerda);
    printf("Theta1_esquerda: %d\n", theta1_esquerda);
    printf("Theta2_esquerda: %d\n", theta2_esquerda);
    printf("Potencia esquerda: %d\n", pot_esquerda);
    printf("Velocidade esquerda modelo: %d\n", velocidade_esquerda_modelo);
}

// Setup e Loop principais
//-----------------------------------------------------------------------------------------------------------

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
    velocidade_Referencia_anterior = velocidade_Referencia;
    velocidade_Referencia = msg->data;
    ROS_INFO("I heard: [%d]", velocidade_Referencia);
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

    while (running)
    {

        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        long int milliseconds = t.tv_sec * 1000;

        printf("TEMPO: %d\n", milliseconds);

        if ((milliseconds / 1000) > 5)
        {
            // printf("PASSOU AQUI NO PRIMEIRO TEMPO\n");

            // if ((milliseconds / 1000) > 30)
            // {
            //     printf("PASSOU NO PRINT DE PARAR\n");
            //     velocidade_Referencia = 0;
            //     parar();
            // }

            if (milliseconds - tempo > 30)
            {
                printf("PASSOU NO SEGUNDO TEMPO\n");

                tempo_aux = (milliseconds - tempo);
                tempo = milliseconds;

                encoder0Pos = encoder(2);
                encoder1Pos = encoder(3);

                voltas_esquerda = encoder0Pos / 4096;
                voltas_direita = encoder1Pos / 4096;

                velocidade_esquerda = 1000 * (voltas_esquerda - voltas_esquerda_anterior) / (tempo_aux);
                velocidade_direita = 1000 * (voltas_direita - voltas_direita_anterior) / (tempo_aux);

                voltas_esquerda_anterior = voltas_esquerda;
                voltas_direita_anterior = voltas_direita;

                printf("Velocidade esquerda: %d\n", velocidade_esquerda);
                printf("Velocidade direita: %d\n", velocidade_direita);
            }

            controleAdaptativoVelocidade();

            ros::init(argc, argv, "listener");
            ros::NodeHandle n;

            ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

            ros::spin();

            printf("Velocidade recebida: %f\n", velocidade_Referencia);

            printf("Contador encoder esquerdo: %d\n", encoder0Pos);
            printf("Contador encoder direito: %d\n", encoder1Pos);
            printf("Potencia direita: %f\n", pot_direita);
            printf("Potencia esquerda: %f\n", pot_esquerda);
            printf("-----------------------------------------");
        }
    }

    rc_cleanup();
    rc_pwm_cleanup(0);
    rc_encoder_eqep_cleanup();

    return 0;
}
