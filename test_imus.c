#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include "mpu6050.h"

int file;
const char *bus = "/dev/i2c-1";

// --- The Device Object ---
// This struct holds everything unique to one sensor.
// It acts as your "Software Buffer" holding the latest state.
typedef struct {
    int addr;           // I2C Address
    float a_scale;      // Current Accel sensitivity divider
    float g_scale;      // Current Gyro sensitivity divider
    
    // Processed Data
    float ax_g, ay_g, az_g;
    float gx_ds, gy_ds, gz_ds;
    float temp_c;
} MPU6050_Device;

// Function to configure a specific IMU
void setup_imu(MPU6050_Device *dev, uint8_t a_range, uint8_t g_range, uint8_t dlpf) {
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) {
        printf("Failed to connect to IMU at 0x%02x\n", dev->addr);
        return;
    }

    // 1. Wake Up
    uint8_t wake[2] = {PWR_MGMT_1, 0x00};
    write(file, wake, 2);

    // 2. Set Accelerometer Range
    uint8_t a_conf[2] = {ACCEL_CONFIG, a_range};
    write(file, a_conf, 2);

    // Set the internal math scale based on your choice
    switch(a_range) {
        case ACCEL_FS_2G:  dev->a_scale = 16384.0; break;
        case ACCEL_FS_4G:  dev->a_scale = 8192.0;  break;
        case ACCEL_FS_8G:  dev->a_scale = 4096.0;  break;
        case ACCEL_FS_16G: dev->a_scale = 2048.0;  break;
        default: dev->a_scale = 16384.0;
    }

    // 3. Set Gyro Range
    uint8_t g_conf[2] = {GYRO_CONFIG, g_range};
    write(file, g_conf, 2);

    switch(g_range) {
        case GYRO_FS_250:  dev->g_scale = 131.0; break;
        case GYRO_FS_500:  dev->g_scale = 65.5;  break;
        case GYRO_FS_1000: dev->g_scale = 32.8;  break;
        case GYRO_FS_2000: dev->g_scale = 16.4;  break;
        default: dev->g_scale = 131.0;
    }

    // 4. Set DLPF (Bandwidth)
    uint8_t d_conf[2] = {CONFIG, dlpf};
    write(file, d_conf, 2);
    
    // 5. Set Sample Rate Divider to 0 (Max Speed)
    uint8_t rate[2] = {SMPLRT_DIV, 0x00};
    write(file, rate, 2);

    printf("IMU 0x%02x Configured: A-Scale=%.1f, G-Scale=%.1f\n", dev->addr, dev->a_scale, dev->g_scale);
}

// Function to read all data into the device struct
void read_imu_data(MPU6050_Device *dev) {
    uint8_t buffer[14]; // The "Hardware Buffer"
    uint8_t start_reg = ACCEL_XOUT_H;

    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) return;

    write(file, &start_reg, 1);

    if (read(file, buffer, 14) == 14) {
        // 1. Raw Byte Assembly
        int16_t raw_ax = (buffer[0] << 8) | buffer[1];
        int16_t raw_ay = (buffer[2] << 8) | buffer[3];
        int16_t raw_az = (buffer[4] << 8) | buffer[5];
        int16_t raw_temp = (buffer[6] << 8) | buffer[7];
        int16_t raw_gx = (buffer[8] << 8) | buffer[9];
        int16_t raw_gy = (buffer[10] << 8) | buffer[11];
        int16_t raw_gz = (buffer[12] << 8) | buffer[13];

        // 2. Conversion to Real Units (using the stored scales)
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
        perror("Failed to open I2C");
        return 1;
    }

    // --- SETUP: Define your Sensors here ---
    // Create the struct instances
    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };

    // Configure IMU 1 (Standard Robot Settings)
    // 4G Range, 500 deg/s Gyro, 44Hz Filter
    setup_imu(&imu1, ACCEL_FS_4G, GYRO_FS_500, DLPF_44HZ);

    // Configure IMU 2 (High Impact / Fast Spin Settings)
    // 8G Range, 2000 deg/s Gyro, 94Hz Filter
    setup_imu(&imu2, ACCEL_FS_8G, GYRO_FS_2000, DLPF_94HZ);

    printf("\nReading All Sensors... Press Ctrl+C to stop.\n");
    
    while(1) {
        // Pull buffer from hardware
        read_imu_data(&imu1);
        read_imu_data(&imu2);

        // Print Unified Data
        // \033[2J clears screen, \033[H moves cursor to top (Linux/Unix terminal codes)
        printf("\033[2J\033[H"); 
        
        printf("=== IMU 1 (0x68) === [Temp: %.1f C]\n", imu1.temp_c);
        printf("ACCEL (G):  X:%6.2f  Y:%6.2f  Z:%6.2f\n", imu1.ax_g, imu1.ay_g, imu1.az_g);
        printf("GYRO (d/s): X:%6.1f  Y:%6.1f  Z:%6.1f\n", imu1.gx_ds, imu1.gy_ds, imu1.gz_ds);
        
        printf("\n=== IMU 2 (0x69) === [Temp: %.1f C]\n", imu2.temp_c);
        printf("ACCEL (G):  X:%6.2f  Y:%6.2f  Z:%6.2f\n", imu2.ax_g, imu2.ay_g, imu2.az_g);
        printf("GYRO (d/s): X:%6.1f  Y:%6.1f  Z:%6.1f\n", imu2.gx_ds, imu2.gy_ds, imu2.gz_ds);

        usleep(100000); // 10Hz Refresh for readability
    }

    return 0;
}