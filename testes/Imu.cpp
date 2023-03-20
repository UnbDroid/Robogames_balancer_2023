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

ros::Publisher pub;

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

void dmp_callback(void)
{
    static ros::Time last_publish_time = ros::Time::now();

    ros::Time current_time = ros::Time::now();
    if ((current_time - last_publish_time).toSec() >= 1.0 / PUBLISH_RATE_HZ)
    {
        std_msgs::Float32 msg;
        msg.data = (data.dmp_TaitBryan[TB_ROLL_Y] * RAD_TO_DEG);
        pub.publish(msg);
        // ROS_INFO("%f", msg.data);
        last_publish_time = current_time;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Imu");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("angle", 10);
    ros::Rate rate(PUBLISH_RATE_HZ);

    signal(SIGINT, __signal_handler);
    running = 1;

    rc_mpu_config_t conf = rc_mpu_default_config();
    conf.i2c_bus = I2C_BUS;
    conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
    conf.dmp_sample_rate = SAMPLE_RATE_HZ;
    conf.orient = ORIENTATION_Y_UP;
    // if gyro isn't calibrated, run the calibration routine
    if (!rc_mpu_is_gyro_calibrated())
    {
        printf("Gyro not calibrated, automatically starting calibration routine\n");
        printf("Let your MiP sit still on a firm surface\n");
        rc_mpu_calibrate_gyro_routine(conf);
    }

    if (rc_mpu_initialize_dmp(&data, conf))
    {
        printf("rc_mpu_initialize_failed\n");
        return -1;
    }
    rc_mpu_set_dmp_callback(dmp_callback);

    printf("IMU started");

    while (ros::ok())
    {
        // usleep(1000);

        ros::spinOnce();

        rate.sleep();
    }

    rc_mpu_set_dmp_callback(NULL);
    rc_mpu_power_off();
    fflush(stdout);
}