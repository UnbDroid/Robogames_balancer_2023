#include <robotcontrol.h>
#include <stdio.h>
#include <linux/gpio.h>
#include <rc/pwm.h>
#include <rc/pinmux.h>


#define GPIO_PIN_3_1 1 // SOUT
#define GPIO_PIN_3_2 2 //S3


#define UART1_HEADER_PIN_3   14 // S2
#define UART1_HEADER_PIN_4   15 // S1

unsigned int pulseIn(int gpio, int level){
    struct timeval t0;
    struct timeval t1;
    gettimeofday(&t0, NULL);
    while (rc_gpio_get_value(3, gpio) != level) {
        gettimeofday(&t0, NULL);
    }
    while (rc_gpio_get_value(3, gpio) == level) {
        gettimeofday(&t1, NULL);
    }
    unsigned int pulse_us = (t1.tv_sec - t0.tv_sec) * 1000000 + (t1.tv_usec - t0.tv_usec);
    return pulse_us;
}


void lerCores(){

    rc_gpio_set_value(3, UART1_HEADER_PIN_3, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_2, 0);
    rc_usleep(50000);

    // Lê a frequencia de saída do fotodiodo vermelho
    int red = pulseIn(GPIO_PIN_3_1, 1);

    // Configura a leitura para os fotodiodos Green (Verde)
    rc_gpio_set_value(3, UART1_HEADER_PIN_3, 1);
    rc_gpio_set_value(3, GPIO_PIN_3_2, 1);
    rc_usleep(50000);

    // Lê a frequencia de saída do fotodiodo verde
    int green = pulseIn(GPIO_PIN_3_1, 1);

    // Configura a leitura para os fotodiodos Blue (Azul)
    rc_gpio_set_value(3, UART1_HEADER_PIN_3, 0);
    rc_gpio_set_value(3, GPIO_PIN_3_2, 1);
    rc_usleep(50000);

    // Lê a frequencia de saída do fotodiodo azul
    int blue = pulseIn(GPIO_PIN_3_1, 1);

    printf("RGB [%d] [%d] [%d] \n", red, green, blue);
}

int main(int argc, char *argv[])
{

    rc_pinmux_set(UART1_HEADER_PIN_3, PINMUX_GPIO);
    rc_gpio_init(3, UART1_HEADER_PIN_3, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_2, GPIOHANDLE_REQUEST_OUTPUT);
    rc_gpio_init(3, GPIO_PIN_3_1, GPIOHANDLE_REQUEST_INPUT);

    while (1)
    {
        lerCores();
    }    

}

