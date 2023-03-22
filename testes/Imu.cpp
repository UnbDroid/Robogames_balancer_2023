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
#define SAMPLE_RATE_HZ 50
#define PERIODO 30000 // TODO microssegundos
#define PUBLISH_RATE_HZ 50

static rc_mpu_data_t mpu_data;
static rc_mpu_data_t mpu_data_theta;

ros::Publisher pub, pub_thetha_ponto;

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

        // ROS_INFO("%f", msg.data);
    }
    last_publish_time = current_time;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Imu");
    ros::NodeHandle n;
    pub = n.advertise<std_msgs::Float32>("angle", 10);
    pub_thetha_ponto = n.advertise<std_msgs::Float32>("angle_theta_ponto", 10);
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

        if(rc_mpu_initialize(&mpu_data_theta, conf)){
                fprintf(stderr,"rc_mpu_initialize_failed\n");
                return -1;
        }

    if (rc_mpu_initialize_dmp(&mpu_data, conf))
    {
        printf("rc_mpu_initialize_dmp_failed\n");
        return -1;
    }

    rc_mpu_set_dmp_callback(&dmp_callback);

    while (ros::ok())
    {
        static ros::Time last_publish_time_theta = ros::Time::now();

        ros::Time current_time_theta = ros::Time::now();
        if ((current_time_theta - last_publish_time_theta).toSec() >= 1.0 / PUBLISH_RATE_HZ)
        {
            if (rc_mpu_read_gyro(&mpu_data_theta) < 0)
            {
                printf("read gyro data failed\n");
            }

            std_msgs::Float32 msgThetaPonto;
            msgThetaPonto.data = (mpu_data_theta.gyro[1] * DEG_TO_RAD);
            pub_thetha_ponto.publish(msgThetaPonto);
            // ROS_INFO("%f", msgThetaPonto.data);
        }

        last_publish_time_theta = current_time_theta;

        ros::spinOnce();

        rate.sleep();
    }

    rc_mpu_set_dmp_callback(NULL);
    rc_mpu_power_off();
    fflush(stdout);
}