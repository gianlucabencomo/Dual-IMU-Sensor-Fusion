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

typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} IMUData;

void read_imu(int addr, IMUData *data) {
    uint8_t buffer[14];
    uint8_t start_reg = ACCEL_XOUT_H;

    // Switch to the correct device address first!
    if (ioctl(file, I2C_SLAVE, addr) < 0) return;

    write(file, &start_reg, 1);

    if (read(file, buffer, 14) == 14) {
        // Names must match the struct exactly: ax, ay, az...
        data->ax = (int16_t)((buffer[0] << 8) | buffer[1]);
        data->ay = (int16_t)((buffer[2] << 8) | buffer[3]);
        data->az = (int16_t)((buffer[4] << 8) | buffer[5]);
        data->temp = (int16_t)((buffer[6] << 8) | buffer[7]);
        data->gx = (int16_t)((buffer[8] << 8) | buffer[9]);
        data->gy = (int16_t)((buffer[10] << 8) | buffer[11]);
        data->gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    }
}

void init_imu(int addr) {
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        printf("Error: Could not select IMU at 0x%02x\n", addr);
        return;
    }
    
    uint8_t wake_buf[2] = {PWR_MGMT_1, 0x00};
    write(file, wake_buf, 2);

    uint8_t raw_conf[2] = {CONFIG, 0x00}; 
    write(file, raw_conf, 2);

    uint8_t rate_buf[2] = {SMPLRT_DIV, 0x00};
    write(file, rate_buf, 2); 
}

int main() {
    IMUData imu1, imu2;

    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    init_imu(IMU1_ADDR); // 0x68
    init_imu(IMU2_ADDR); // 0x69

    printf("Starting Raw Read (20Hz loop)...\n");
    printf("IMU1 (0x68) Accel X | IMU2 (0x69) Accel X\n");
    printf("-----------------------------------------\n");

    while (1) {
        read_imu(IMU1_ADDR, &imu1);
        read_imu(IMU2_ADDR, &imu2);

        // Raw LSB divided by 16384 for +/- 2g range
        float ax1 = imu1.ax / 16384.0;
        float ax2 = imu2.ax / 16384.0;

        printf("\r %8.2f G      |  %8.2f G      ", ax1, ax2);
        fflush(stdout);
        
        usleep(50000); // Wait 50ms
    }
    return 0;
}