#!/usr/bin/env python

import time
import smbus
import struct
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from mpu6050_pkg.msg import Rpy
from tf.transformations import quaternion_about_axis, euler_from_quaternion


PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

TEMP_H = 0x41
TEMP_L = 0x42

ADDR = None
bus = None
IMU_FRAME = None

roll = pitch = yaw = 0
M_PI = 3.141593

# read_word and read_word_2c

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)


def publish_imu(timer_event):
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME
    rpy_msg = Rpy()

    #ivme degerlerini okuma
    #2g'lik varsayilan hassasiyette 16384 ile olceklendirmemiz gerekiyor
    #Imu msg dosyasinda ivme (m/s^2) formatinda oldugundan dolayi 9.807 ile carpmamiz gerekmekte.

    rescale = 16384.0 / 9.807

    accel_x = read_word_2c(ACCEL_XOUT_H) / rescale
    accel_y = read_word_2c(ACCEL_YOUT_H) / rescale
    accel_z = read_word_2c(ACCEL_ZOUT_H) / rescale

    # Quaternion formunda oryantasyon hesabi
    accel = accel_x, accel_y, accel_z
    ref = np.array([0, 0, 1])
    acceln = accel / np.linalg.norm(accel)
    axis = np.cross(acceln, ref)
    angle = np.arccos(np.dot(acceln, ref))
    orientation = quaternion_about_axis(angle, axis)

    # Gyro degerlerini okuma 

    gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
    gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
    gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0
    
    # IMU mesajini olusturma
    o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation

    orientation_list = [o.x, o.y, o.z, o.w]

    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()
    rpy_msg.header.stamp = rospy.Time.now()

    rpy_msg.roll = roll * 180/M_PI
    rpy_msg.pitch = pitch * 180/M_PI
    rpy_msg.yaw = yaw * 180/M_PI

    imu_pub.publish(imu_msg)
    rpy_pub.publish(rpy_msg)


temp_pub = None
imu_pub = None
rpy_pub = None

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)

    temp_pub = rospy.Publisher('temperature', Temperature,queue_size = 10)
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size = 10)
    rpy_pub = rospy.Publisher('rpy/data', Rpy, queue_size = 10)
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    rospy.spin()

