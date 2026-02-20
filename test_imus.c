#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdint.h>
#include "mpu6050.h"

int file;
const char *bus = "/dev/i2c-1";

typedef struct {
    int addr;           
    float a_scale;      
    float g_scale;      
    
    // --- NEW: Calibration Offsets ---
    float gx_offset, gy_offset, gz_offset;

    float ax_g, ay_g, az_g;
    float gx_ds, gy_ds, gz_ds;
    float temp_c;
} MPU6050_Device;

int setup_imu(MPU6050_Device *dev, uint8_t a_range, uint8_t g_range, uint8_t dlpf) {
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) {
        printf("System Error: Failed to connect to I2C bus for IMU at 0x%02x\n", dev->addr);
        return 0;
    }

    int who_am_i = i2c_smbus_read_byte_data(file, WHO_AM_I);
    if (who_am_i != 0x68) {
        printf("Hardware Error: IMU at 0x%02x returned WHO_AM_I = 0x%02x (Expected 0x68). Check wiring!\n", dev->addr, who_am_i);
        return 0; 
    }
    
    i2c_smbus_write_byte_data(file, PWR_MGMT_1, 0x00);
    i2c_smbus_write_byte_data(file, ACCEL_CONFIG, a_range);
    
    switch(a_range) {
        case ACCEL_FS_2G:  dev->a_scale = 16384.0; break;
        case ACCEL_FS_4G:  dev->a_scale = 8192.0;  break;
        case ACCEL_FS_8G:  dev->a_scale = 4096.0;  break;
        case ACCEL_FS_16G: dev->a_scale = 2048.0;  break;
        default: dev->a_scale = 16384.0;
    }

    i2c_smbus_write_byte_data(file, GYRO_CONFIG, g_range);
    switch(g_range) {
        case GYRO_FS_250:  dev->g_scale = 131.0; break;
        case GYRO_FS_500:  dev->g_scale = 65.5;  break;
        case GYRO_FS_1000: dev->g_scale = 32.8;  break;
        case GYRO_FS_2000: dev->g_scale = 16.4;  break;
        default: dev->g_scale = 131.0;
    }

    i2c_smbus_write_byte_data(file, CONFIG, dlpf);
    i2c_smbus_write_byte_data(file, SMPLRT_DIV, 0x01); 

    // Initialize offsets to zero by default
    dev->gx_offset = 0; dev->gy_offset = 0; dev->gz_offset = 0;

    printf("IMU 0x%02x Configured: A-Scale=%.1f, G-Scale=%.1f\n", dev->addr, dev->a_scale, dev->g_scale);
    return 1;
}

// Function to read data (Now includes offset subtraction)
void read_imu_data(MPU6050_Device *dev) {
    uint8_t buffer[14]; 

    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) return;

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

        // --- NEW: Subtract the loaded offsets to zero out the gyro ---
        dev->gx_ds = (raw_gx / dev->g_scale) - dev->gx_offset;
        dev->gy_ds = (raw_gy / dev->g_scale) - dev->gy_offset;
        dev->gz_ds = (raw_gz / dev->g_scale) - dev->gz_offset;
    }
}

// --- NEW: Calibration Functions ---

void calibrate_gyro(MPU6050_Device *dev) {
    printf(">> Calibrating IMU 0x%02x. DO NOT MOVE SENSOR...\n", dev->addr);
    sleep(2); // Give the user 2 seconds to take their hands off the table

    int num_samples = 1000;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    // Temporarily clear offsets so we read pure hardware data
    dev->gx_offset = 0; dev->gy_offset = 0; dev->gz_offset = 0;

    for (int i = 0; i < num_samples; i++) {
        read_imu_data(dev);
        sum_x += dev->gx_ds;
        sum_y += dev->gy_ds;
        sum_z += dev->gz_ds;
        usleep(2000); // Wait 2ms (500Hz)
    }

    // Calculate the average drift
    dev->gx_offset = sum_x / num_samples;
    dev->gy_offset = sum_y / num_samples;
    dev->gz_offset = sum_z / num_samples;

    // Save to a text file specific to this I2C address
    char filename[32];
    snprintf(filename, sizeof(filename), "imu_cal_0x%02x.txt", dev->addr);
    FILE *f = fopen(filename, "w");
    if (f) {
        fprintf(f, "%f %f %f\n", dev->gx_offset, dev->gy_offset, dev->gz_offset);
        fclose(f);
        printf(">> Saved offsets for 0x%02x: X:%.2f Y:%.2f Z:%.2f\n", dev->addr, dev->gx_offset, dev->gy_offset, dev->gz_offset);
    } else {
        printf(">> Error: Could not save calibration file for 0x%02x.\n", dev->addr);
    }
}

int load_calibration(MPU6050_Device *dev) {
    char filename[32];
    snprintf(filename, sizeof(filename), "imu_cal_0x%02x.txt", dev->addr);
    
    FILE *f = fopen(filename, "r");
    if (f) {
        fscanf(f, "%f %f %f", &dev->gx_offset, &dev->gy_offset, &dev->gz_offset);
        fclose(f);
        printf(">> Loaded saved calibration for 0x%02x.\n", dev->addr);
        return 1; // Success
    }
    return 0; // File not found
}

// ----------------------------------

int main() {
    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };

    printf("--- Initializing Hardware ---\n");
    if (!setup_imu(&imu1, ACCEL_FS_8G, GYRO_FS_1000, DLPF_94HZ)) return 1;
    if (!setup_imu(&imu2, ACCEL_FS_8G, GYRO_FS_1000, DLPF_94HZ)) return 1;

    printf("\n--- Gyroscope Calibration ---\n");
    // Try to load the file. If it fails (returns 0), run the calibration routine.
    if (!load_calibration(&imu1)) calibrate_gyro(&imu1);
    if (!load_calibration(&imu2)) calibrate_gyro(&imu2);

    printf("\nReading All Sensors at 500Hz... Press Ctrl+C to stop.\n");
    sleep(1); 
    
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