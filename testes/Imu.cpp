//Includes ------------------------------------------------------------------------------------------------------------------

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

//Defines ------------------------------------------------------------------------------------------------------------------------------------

#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21
#define SAMPLE_RATE_HZ 50
#define PUBLISH_RATE_HZ 70
#define QUEUE_SIZE 5000

static rc_mpu_data_t mpu_data;

static float prev_angle = 0.0;
static ros::Time prev_time;

ros::Publisher pub, pub_theta_ponto;

static void dmp_callback(void)
{
    static ros::Time last_publish_time = ros::Time::now();

    ros::Time current_time = ros::Time::now();

    ros::Duration dt = current_time - last_publish_time;
    if ((dt).toSec() >= 1.0 / PUBLISH_RATE_HZ)
    {
        std_msgs::Float32 msg;
        // msg.data = (data.dmp_TaitBryan[TB_ROLL_Y] * RAD_TO_DEG);
        msg.data = (mpu_data.dmp_TaitBryan[TB_ROLL_Y]);
        pub.publish(msg);

        if (prev_time.isZero())
        {
            prev_time = current_time;
        }
        else
        {
            ros::Duration dt = current_time - prev_time;
            if (dt.toSec() > 0.0)
            {
                float d_angle = msg.data - prev_angle;
                float angle_derivative = d_angle / dt.toSec();

                std_msgs::Float32 angle_derivative_msg;
                angle_derivative_msg.data = angle_derivative;
                pub_theta_ponto.publish(angle_derivative_msg);
            }
        }
        prev_angle = msg.data;
        prev_time = current_time;
    }
    last_publish_time = current_time;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Imu");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("angle", QUEUE_SIZE);
    pub_theta_ponto = n.advertise<std_msgs::Float32>("angle_theta_ponto", QUEUE_SIZE);
    ros::Rate rate(PUBLISH_RATE_HZ);

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

    if (rc_mpu_initialize_dmp(&mpu_data, conf))
    {
        printf("rc_mpu_initialize_dmp_failed\n");
        return -1;
    }

    rc_mpu_set_dmp_callback(&dmp_callback);

    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    rc_mpu_set_dmp_callback(NULL);
    rc_mpu_power_off();
    fflush(stdout);
}