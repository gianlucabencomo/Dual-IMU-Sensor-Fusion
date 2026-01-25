#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

// Power Management
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75

// Configuration Registers
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C

// I2C Addresses
#define IMU1_ADDR 0x68 // IMU 1
#define IMU2_ADDR 0x69 // IMU 2

// Accelerometer (High and Low)
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// Temperature Sensor
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42

// Gyroscope (High and Low)
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#endif