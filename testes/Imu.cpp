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
#define SAMPLE_RATE_HZ 200

static int running = 0;
static rc_mpu_data_t data;

ros::Publisher pub;

static void __signal_handler(__attribute__((unused)) int dummy)
{
    running = 0;
    return;
}

void dmp_callback(void)
{
    std_msgs::Float32 msg;
    msg.data = (data.dmp_TaitBryan[TB_ROLL_Y] * RAD_TO_DEG);
    pub.publish(msg);
    // ROS_INFO("%f", msg.data);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Imu");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("angle", 10);

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

    while (ros::ok())
    {
        ros::spinOnce();
    }
    rc_mpu_power_off();
    fflush(stdout);
}