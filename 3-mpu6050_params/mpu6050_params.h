#ifndef _MPU6050_PARAMS_H
#define _MPU6050_PARAMS_H

#include <linux/ioctl.h>

#define MPU_IOC_MAGIC     'M'

#define MPU_IOC_SET_ODR   _IOW(MPU_IOC_MAGIC, 0x01, __u32)
#define MPU_IOC_SET_LPF   _IOW(MPU_IOC_MAGIC, 0x02, __u32)
#define MPU_IOC_SET_FS_A  _IOW(MPU_IOC_MAGIC, 0x03, __u32)
#define MPU_IOC_SET_FS_G  _IOW(MPU_IOC_MAGIC, 0x04, __u32)

#define MPU6050_IOC_MAXNR  0x04

#define DRV_NAME          "mpu6050_params"
#define MPU_I2C_ADDR       0x68

#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_INT_PIN_CFG    0x37
#define REG_INT_ENABLE     0x38
#define REG_INT_STATUS     0x3A
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

#endif