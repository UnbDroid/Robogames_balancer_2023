#include "Motor.h"


void direita(float potEsquerda, float potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 1);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
}

void esquerda(float potEsquerda, float potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 1);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
}

void direita_eixo(float potEsquerda, float potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 1);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 0);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 1);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
}

void esquerda_eixo(float potEsquerda, float potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 1);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 1);
}

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

void tras(int potEsquerda, int potDireita)
{

    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', potEsquerda);
    rc_pwm_set_duty(0, 'B', potDireita);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 1);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 1);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
}

void parar()
{
    // Potência direita e esquerda
    rc_pwm_set_duty(0, 'A', 0.0);
    rc_pwm_set_duty(0, 'B', 0.0);

    // Setar os pinos GPIO LOW e HIGH
    rc_gpio_set_value(1, GPIO_PIN_1_17, 0);
    rc_gpio_set_value(1, GPIO_PIN_1_25, 0);

    // Setar os pinos GPIO HIGH e LOW
    rc_gpio_set_value(3, GPIO_PIN_3_17, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_20, 0);
}

int encoder(int canal)
{
    int encoder = rc_encoder_eqep_read(canal);
    printf("canal: %d, encoder: %d\n", canal, encoder);
    return encoder;
}