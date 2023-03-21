#include <math.h>
#include <robotcontrol.h>
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include <termios.h>
#include <rc/mpu.h>

#include "ros/ros.h"
#include <sstream>
#include "std_msgs/Float32.h"

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
#define SAMPLE_RATE_HZ 100
#define PERIODO 30000 // TODO microssegundos
#define PUBLISH_RATE_HZ 80

int tempo = 0;

static int running = 0;
static rc_mpu_data_t data;

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Imu");
    ros::NodeHandle n;
    ros::Publisher pub_thetha_ponto = n.advertise<std_msgs::Float32>("angle_theta_ponto", 10);
    ros::Rate rate(PUBLISH_RATE_HZ);

    signal(SIGINT, __signal_handler);
    running = 1;

    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.orient = ORIENTATION_Y_UP;
    // if gyro isn't calibrated, run the calibration routine
    if (!rc_mpu_is_gyro_calibrated())
    {
        printf("Gyro not calibrated, automatically starting calibration routine\n");
        printf("Let your MiP sit still on a firm surface\n");
        rc_mpu_calibrate_gyro_routine(conf);
    }

    if (rc_mpu_initialize(&data, conf))
    {
        fprintf(stderr, "rc_mpu_initialize_failed\n");
        return -1;
    }

    while (ros::ok())
    {
        static ros::Time last_publish_time = ros::Time::now();

        ros::Time current_time = ros::Time::now();
        if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            if (rc_mpu_read_gyro(&data) < 0)
            {
                printf("read gyro data failed\n");
            }

            std_msgs::Float32 msgThetaPonto;
            msgThetaPonto.data = (data.gyro[1] * DEG_TO_RAD);
            pub_thetha_ponto.publish(msgThetaPonto);
            ROS_INFO("%f", msgThetaPonto.data);
        }

        ros::spinOnce();

        rate.sleep();
    }

    rc_mpu_power_off();
    fflush(stdout);
}