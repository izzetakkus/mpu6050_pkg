#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "mpu6050_pkg/Rpy.h"
#include <cmath>

#define RAD_TO_DEG 180/M_PI

using namespace std;

const int I2C_ADDR = 0x68;
const int PWR_MGMT_1 = 0x6B;

/*IMU Data*/

double accX, accY, accZ;
double gyroX, gyroY, gyroz;
int16_t tempRaw;

double gyroXangle, gyroYangle;  //Angle calculate using only gyro
double compAngleX, compAngleY;  //Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    //Calculated angle using a Kalman filter

float read_word_2c(int fd, int addr) {
  int high = wiringPiI2CReadReg8(fd, addr);
  int low = wiringPiI2CReadReg8(fd, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

int main(int argc, char **argv) {

  // Connect to device.
  int fd = wiringPiI2CSetup(I2C_ADDR);
  if (fd == -1) {
    printf("no i2c device found?\n");
    return -1;
  }
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(fd, PWR_MGMT_1, 0);

  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu", 10);
  ros::Publisher rpy_pub = node.advertise<mpu6050_pkg::Rpy>("orientation/data", 10);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {

    sensor_msgs::Imu msg;
    mpu6050_pkg::Rpy rpy_msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = '0';  // no frame

    rpy_msg.header.stamp = ros::Time::now();

    // Read gyroscope values.
    // At default sensitivity of 250deg/s we need to scale by 131.
    msg.angular_velocity.x = read_word_2c(fd, 0x43) / 131;
    msg.angular_velocity.y = read_word_2c(fd, 0x45) / 131;
    msg.angular_velocity.z = read_word_2c(fd, 0x47) / 131;

    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
    const float rescale = 16384.0 / 9.807;

    accX = read_word_2c(fd, 0x3b) / rescale;
    accY = read_word_2c(fd, 0x3d) / rescale;
    accZ = read_word_2c(fd, 0x3f) / rescale;

    msg.linear_acceleration.x = accX;
    msg.linear_acceleration.y = accY;
    msg.linear_acceleration.z = accZ;


    rpy_msg.roll = atan(accY / sqrt(pow(accX,2) + pow(accZ,2))) * RAD_TO_DEG;
    rpy_msg.pitch = atan(-accX / sqrt(pow(accY,2) + pow(accZ,2))) * RAD_TO_DEG;


    // Pub & sleep.
    pub.publish(msg);
    rpy_pub.publish(rpy_msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
