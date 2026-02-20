#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

// --- Register Map ---
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN      0x23  // FIFO Enable

// --- I2C Addresses ---
#define IMU1_ADDR 0x68
#define IMU2_ADDR 0x69

// --- Data Registers (Sequential) ---
#define ACCEL_XOUT_H 0x3B
// ... The rest follow sequentially to 0x48 ...

// --- Configuration Constants ---
// Accelerometer Ranges (Register 0x1C)
#define ACCEL_FS_2G  0x00
#define ACCEL_FS_4G  0x08
#define ACCEL_FS_8G  0x10
#define ACCEL_FS_16G 0x18

// Gyroscope Ranges (Register 0x1B)
#define GYRO_FS_250  0x00
#define GYRO_FS_500  0x08
#define GYRO_FS_1000 0x10
#define GYRO_FS_2000 0x18

// DLPF (Digital Low Pass Filter) Bandwidths (Register 0x1A)
#define DLPF_260HZ   0x00 // RAW
#define DLPF_184HZ   0x01
#define DLPF_94HZ    0x02
#define DLPF_44HZ    0x03 // Best for general robotics
#define DLPF_21HZ    0x04
#define DLPF_10HZ    0x05
#define DLPF_5HZ     0x06

// Sampling Rate
#define SAMPLE_1000HZ 0x00
#define SAMPLE_500HZ 0x01
#define SAMPLE_250HZ 0x03
#define SAMPLE_100HZ 0x09

#endif