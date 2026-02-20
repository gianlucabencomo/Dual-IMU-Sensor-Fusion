#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>   // NEW: The SMBus library for safe reads/writes
#include <stdint.h>
#include "mpu6050.h"

int file;
const char *bus = "/dev/i2c-1";

typedef struct {
    int addr;           
    float a_scale;      
    float g_scale;      
    
    float ax_g, ay_g, az_g;
    float gx_ds, gy_ds, gz_ds;
    float temp_c;
} MPU6050_Device;

// Setup function now returns 1 for success, 0 for failure
int setup_imu(MPU6050_Device *dev, uint8_t a_range, uint8_t g_range, uint8_t dlpf) {
    // Connect the OS switchboard to this specific IMU
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) {
        printf("System Error: Failed to connect to I2C bus for IMU at 0x%02x\n", dev->addr);
        return 0;
    }

    // --- 1. PROBING & VERIFICATION ---
    int who_am_i = i2c_smbus_read_byte_data(file, WHO_AM_I);
    if (who_am_i != 0x68) {
        printf("Hardware Error: IMU at 0x%02x returned WHO_AM_I = 0x%02x (Expected 0x68). Check wiring!\n", dev->addr, who_am_i);
        return 0; 
    }
    printf("IMU at 0x%02x successfully detected.\n", dev->addr);

    // --- 2. CONFIGURATION (Using safe SMBus writes) ---
    // Wake Up
    i2c_smbus_write_byte_data(file, PWR_MGMT_1, 0x00);

    // Set Accelerometer Range
    i2c_smbus_write_byte_data(file, ACCEL_CONFIG, a_range);
    switch(a_range) {
        case ACCEL_FS_2G:  dev->a_scale = 16384.0; break;
        case ACCEL_FS_4G:  dev->a_scale = 8192.0;  break;
        case ACCEL_FS_8G:  dev->a_scale = 4096.0;  break;
        case ACCEL_FS_16G: dev->a_scale = 2048.0;  break;
        default: dev->a_scale = 16384.0;
    }

    // Set Gyro Range
    i2c_smbus_write_byte_data(file, GYRO_CONFIG, g_range);
    switch(g_range) {
        case GYRO_FS_250:  dev->g_scale = 131.0; break;
        case GYRO_FS_500:  dev->g_scale = 65.5;  break;
        case GYRO_FS_1000: dev->g_scale = 32.8;  break;
        case GYRO_FS_2000: dev->g_scale = 16.4;  break;
        default: dev->g_scale = 131.0;
    }

    // Set DLPF (Bandwidth) and Sample Rate
    i2c_smbus_write_byte_data(file, CONFIG, dlpf);
    i2c_smbus_write_byte_data(file, SMPLRT_DIV, 0x01); 

    printf("IMU 0x%02x Configured: A-Scale=%.1f, G-Scale=%.1f\n", dev->addr, dev->a_scale, dev->g_scale);
    return 1;
}

// Function to read all data safely
void read_imu_data(MPU6050_Device *dev) {
    uint8_t buffer[14]; 

    // Connect to this specific IMU
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) return;

    // --- ATOMIC BURST READ ---
    // This locks the bus, writes the start register, and reads 14 bytes in one uninterruptible motion
    if (i2c_smbus_read_i2c_block_data(file, ACCEL_XOUT_H, 14, buffer) == 14) {
        
        int16_t raw_ax = (buffer[0] << 8) | buffer[1];
        int16_t raw_ay = (buffer[2] << 8) | buffer[3];
        int16_t raw_az = (buffer[4] << 8) | buffer[5];
        int16_t raw_temp = (buffer[6] << 8) | buffer[7];
        int16_t raw_gx = (buffer[8] << 8) | buffer[9];
        int16_t raw_gy = (buffer[10] << 8) | buffer[11];
        int16_t raw_gz = (buffer[12] << 8) | buffer[13];

        dev->ax_g = raw_ax / dev->a_scale;
        dev->ay_g = raw_ay / dev->a_scale;
        dev->az_g = raw_az / dev->a_scale;

        dev->temp_c = (raw_temp / 340.0) + 36.53;

        dev->gx_ds = raw_gx / dev->g_scale;
        dev->gy_ds = raw_gy / dev->g_scale;
        dev->gz_ds = raw_gz / dev->g_scale;
    }
}

int main() {
    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };

    printf("--- Initializing Hardware ---\n");
    
    // Check if initialization failed. If either fails, safely exit.
    if (!setup_imu(&imu1, ACCEL_FS_8G, GYRO_FS_1000, DLPF_94HZ)) return 1;
    if (!setup_imu(&imu2, ACCEL_FS_8G, GYRO_FS_1000, DLPF_94HZ)) return 1;

    printf("\nReading All Sensors at 500Hz... Press Ctrl+C to stop.\n");
    
    int counter = 0;

    while(1) {
        read_imu_data(&imu1);
        read_imu_data(&imu2);

        if (counter % 50 == 0) {
            printf("\033[2J\033[H"); 
            printf("=== IMU 1 (0x68) === [Temp: %.1f C]\n", imu1.temp_c);
            printf("ACCEL: X:%6.2f Y:%6.2f Z:%6.2f\n", imu1.ax_g, imu1.ay_g, imu1.az_g);
            printf("GYRO:  X:%6.1f Y:%6.1f Z:%6.1f\n", imu1.gx_ds, imu1.gy_ds, imu1.gz_ds);
            
            printf("\n=== IMU 2 (0x69) === [Temp: %.1f C]\n", imu2.temp_c);
            printf("ACCEL: X:%6.2f Y:%6.2f Z:%6.2f\n", imu2.ax_g, imu2.ay_g, imu2.az_g);
            printf("GYRO:  X:%6.1f Y:%6.1f Z:%6.1f\n", imu2.gx_ds, imu2.gy_ds, imu2.gz_ds);
        }

        counter++;
        usleep(2000); 
    }

    return 0;
}