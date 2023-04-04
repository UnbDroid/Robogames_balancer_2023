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

// Funções do ROS

void chatterCallback(const std_msgs::Float32::ConstPtr &msg)
{
    float teste = msg->data;
    velocidade_referencia = teste;
}

static void dmp_callback(void)
{
    static ros::Time last_publish_time_imu = ros::Time::now();

    ros::Time current_time_imu = ros::Time::now();

    ros::Duration dt = current_time_imu - last_publish_time_imu;
    if ((dt).toSec() >= 1.0 / PUBLISH_RATE_HZ)
    {
        theta = (mpu_data.dmp_TaitBryan[TB_ROLL_Y]) + 0.05;

        if (prev_time.isZero())
        {
            prev_time = current_time_imu;
        }
        else
        {
            ros::Duration dt = current_time_imu - prev_time;
            if (dt.toSec() > 0.0)
            {
                float d_angle = theta - prev_angle;
                float angle_derivative = d_angle / dt.toSec();
                omega = angle_derivative;
            }
        }
        prev_angle = theta;
        prev_time = current_time_imu;
    }
    last_publish_time_imu = current_time_imu;
}

void referenciaPosicaoCallback(const std_msgs::Float32::ConstPtr &msg)
{
    referencePosicao = msg->data;
}

void referenciaVelocidadeCallback(const std_msgs::Float32::ConstPtr &msg)
{
    referenceVelocidade = msg->data;
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
    // ros::Publisher velocidade = n.advertise<std_msgs::Float32>("velocidade", QUEUE_SIZE);
    // ros::Publisher posicao = n.advertise<std_msgs::Float64>("posicao", QUEUE_SIZE);
    // ros::Subscriber sub = n.subscribe("referencia", QUEUE_SIZE, chatterCallback);
    // ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 10);

    // Seta frequência de publicação e escuta
    // ros::Rate rate(PUBLISH_RATE_HZ);

    // ros::init(argc, argv, "Imu");
    // ros::NodeHandle n;
    // pub = n.advertise<std_msgs::Float32>("angle", QUEUE_SIZE);
    // pub_theta_ponto = n.advertise<std_msgs::Float32>("angle_theta_ponto", QUEUE_SIZE);
    ros::Subscriber referenciaPosicao = n.subscribe("referencia_posicao", QUEUE_SIZE, referenciaPosicaoCallback);
    ros::Subscriber referenciaVelocidade = n.subscribe("referencia_velocidade", QUEUE_SIZE, referenciaVelocidadeCallback);
    ros::Rate rate(PUBLISH_RATE_HZ);

    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_sample_rate = SAMPLE_RATE_HZ;
    conf.dmp_auto_calibrate_gyro = 1;
    conf.orient = ORIENTATION_Z_UP;

    if (!rc_mpu_is_gyro_calibrated())
    {
        printf("Gyro not calibrated, automatically starting calibration routine\n");
        printf("Let your MiP sit still on a firm surface\n");
        rc_mpu_calibrate_gyro_routine(conf);
    }

    if (!rc_mpu_is_accel_calibrated())
    {
        printf("Accel not calibrated, automatically starting calibration routine\n");
        rc_mpu_calibrate_accel_routine(conf);
    }

    if (rc_mpu_initialize_dmp(&mpu_data, conf))
    {
        printf("rc_mpu_initialize_dmp_failed\n");
        return -1;
    }

    // Funcionou um metro
    // float K[6] = {15, 0.2690, 2.3, 0.85, 0, 0.00008};
    // float K[6] = {15, 0.2690, 3.5, 0.85, 0, 0.00008};
    float K[6] = {15.5, 0.2690, 3.5, 0.85, 0, 0.000081};
    // float K[6] = {16.5, 0.2690, 3.8, 0.81, 0, 0.000081};
    // float K[6] = {15, 0.2690, 2.1, 0.85, 0, 0.00008};
    // float K[6] = {15, 0.2690, 1.8, 0.85, 0, 0.00008};
    // float K[6] = {16, 0.2690, 1.9, 0.81, 0, 0.00008};

    // float K[6] = {14, 0.4, 1.9, 0.75, 0, 0.00009};

    float error[6];
    float iPosicao = 0;
    float aceleracao = 0;
    float velocidade_old = 0;
    float referencePosicaoTeto = 0;

    static ros::Time last_publish_time = ros::Time::now();

    int count = 0;
    double total_period = 0.0;

    float velocidade_esquerda[MEDIA_MODEL_SIZE] = {0};
    float velocidade_direita[MEDIA_MODEL_SIZE] = {0};

    int count_velocidade = 0;

    float velocidade_esquerda_media = 0;
    float velocidade_direita_media = 0;

    rc_mpu_set_dmp_callback(&dmp_callback);

    for (int i = 0; i < 400000; i++)
    {
        rc_usleep(1);
    }

    printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \n", K[0], K[1], K[2], K[3], K[4], K[5], 0, 0, 0, 0, 0, 0);

    while (ros::ok())
    {

        ros::Time current_time = ros::Time::now();

        ros::Duration dt = current_time - last_publish_time;

        count++;
        total_period += dt.toSec();

        if ((dt).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            referencePosicao += referenceVelocidade * (dt).toSec();

            // referencePosicaoTeto += referenceVelocidade *(dt).toSec();
            // referencePosicao = posicao + 0.05;

            // if(referencePosicao>referencePosicaoTeto){
            //     referencePosicao = referencePosicaoTeto;
            // }

            if (referencePosicao >= 7.0)
            {
                referenceVelocidade = 0;
            }

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
            // float velocidade_metros = velocidadeParaMetros(velocidade_media);
            // velocidade = velocidade_metros;
            velocidade_old = velocidade;
            velocidade = velocidade_media;

            iPosicao += posicao * (dt).toSec();

            aceleracao = (velocidade - velocidade_old) / (dt).toSec();

            error[0] = theta - referenceTheta;
            error[1] = omega - referenceOmega;
            error[2] = posicao - referencePosicao;
            error[3] = velocidade - referenceVelocidade;
            error[4] = iPosicao - 0;
            error[5] = aceleracao - 0;

            // printf("%f, %f, %f, %f \n", theta, omega, posicao, velocidade);

            // Controlador
            referencia = 0;

            for (int i = 0; i < 6; i++)
            {
                if (i != 0)
                {
                    if (error[i] * K[i] >= MAX_ERRO_POS)
                    {
                        referencia += MAX_ERRO_POS;
                    }
                    else
                    {
                        if (error[i] * K[i] <= -MAX_ERRO_POS)
                        {
                            referencia -= MAX_ERRO_POS;
                        }
                        else
                        {
                            referencia += error[i] * K[i];
                        }
                    }
                }
                else
                {
                    referencia += error[i] * K[i];
                }
            }

            // if (theta < 0)
            // {
            //     velocidade_referencia = -1.8;
            // }
            // else
            // {
            //     velocidade_referencia = 1.8;
            // }

            float thetaReferencia = error[0] * K[0];
            float omegaReferencia = error[1] * K[1];
            float posicaoReferencia = error[2] * K[2];
            float velocidadeReferencia = error[3] * K[3];
            float iPosicaoReferencia = error[4] * K[4];
            float aceleracaoReferencia = error[5] * K[5];

            velocidade_referencia = referencia;
            // printf("%f, %f, %f, %f\n", dt.toSec(), velocidade_direita_media, velocidade_esquerda_media, velocidade_referencia);
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", thetaReferencia, omegaReferencia, posicaoReferencia, velocidadeReferencia, aceleracaoReferencia, theta, omega, posicao, velocidade, aceleracao, referencePosicao, referenceVelocidade);
            // printf("%f, %f, %f, %f\n", theta, omega, velocidade, posicao);
            // printf("%f\n", theta);
            // printf("%f, %f\n", theta, omega);
            // printf("%f, %f\n", theta, thetaReferencia);
            // printf("%f, %f, %f, %f\n", posicao, referencePosicao, velocidade, referenceVelocidade);

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

            // printf("%f, %f\n", velocidade, referencia);
            // printf("%f\n", velocidade);
        }
        else
        {
            // rc_led_set(RC_LED_BAT100, 1);
        }

        // rc_led_set(RC_LED_BAT100, 0);
        last_publish_time = current_time;

        // if (count == 1000)
        // {
        //     printf("%f\n", total_period / count);

        //     count = 0;
        //     total_period = 0.0;
        // }

        rate.sleep();
        ros::spinOnce();

        // printf("Potência Esquerda: %f, Velocidade esquerda: %f, encoder esquerda: %d, referência: %f \n", potencia_motor_esquerda, velocidade_esquerda, encoder0Pos, velocidade_referencia);
        // printf("Potência Direita: %f, Velocidade direita: %f, encoder direita: %d, referência: %f \n", potencia_motor_direita, velocidade_direita, encoder1Pos, velocidade_referencia);
        // printf("velocidade metros: %f, posicao metros: %f \n", velocidade_metros, voltas_metros);
    }

    rc_cleanup();
    rc_encoder_eqep_cleanup();

    rc_mpu_set_dmp_callback(NULL);
    rc_mpu_power_off();

    return 0;
}
