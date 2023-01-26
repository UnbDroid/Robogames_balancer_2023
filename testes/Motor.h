#ifndef MOTOR_H
#define MOTOR_H

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

#define GPS_HEADER_PIN_PWM_DIREITA_3 2
#define GPS_HEADER_PIN_PWM_ESQUERDA_4 3
#define GPIO_PIN_1_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_1_25 25 // Set the GPIO pin for the motor control

#define GPIO_PIN_3_17 17 // Set the GPIO pin for the motor control
#define GPIO_PIN_3_20 20


void frente(int potEsquerda, int potDireita);
int encoder(int canal);

#endif