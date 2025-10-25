// mpu6050_params.h
// SPDX-License-Identifier: GPL-2.0
#ifndef __MPU6050_PARAMS_H__
#define __MPU6050_PARAMS_H__

/* ---- Driver name & I2C addr ---- */
#define DRV_NAME        "mpu6050_irq"
#define MPU_I2C_ADDR    0x68

/* ---- MPU6050 Registers ---- */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_INT_PIN_CFG   0x37
#define REG_INT_ENABLE    0x38
#define REG_INT_STATUS    0x3A
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#include <linux/ioctl.h>
#define MPU_IOC_MAGIC    'm'

/* 파라미터: 모두 value 하나 */
#define MPU_IOC_SET_ODR    _IOW(MPU_IOC_MAGIC, 0x01, __u32) /* Output Data Rate(Hz) 요청 */
#define MPU_IOC_SET_LPF    _IOW(MPU_IOC_MAGIC, 0x02, __u32) /* DLPF 0..6 */
#define MPU_IOC_SET_FS_A   _IOW(MPU_IOC_MAGIC, 0x03, __u32) /* 2/4/8/16 g */
#define MPU_IOC_SET_FS_G   _IOW(MPU_IOC_MAGIC, 0x04, __u32) /* 250/500/1000/2000 dps */
/* 새로 추가: IRQ 사용 on/off (1/0) */
#define MPU_IOC_SET_IRQ_EN _IOW(MPU_IOC_MAGIC, 0x05, __u32)

#define MPU6050_IOC_MAXNR  0x05

#endif /* __MPU6050_PARAMS_H__ */
