#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>

int file;
const char *bus = "/dev/i2c-1";

// Register Addresses
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

void init_imu(int addr) {
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        printf("Failed to acquire bus access to 0x%x\n", addr);
        return;
    }
    // Wake up the IMU (write 0 to the power management register)
    uint8_t buf[2] = {PWR_MGMT_1, 0};
    write(file, buf, 2);
}

int16_t read_sensor(int addr, uint8_t reg) {
    uint8_t buf[2];
    
    // Set the address we want to talk to
    ioctl(file, I2C_SLAVE, addr);
    
    // Tell the sensor which register we want to read
    write(file, &reg, 1);
    
    // Read 2 bytes of data
    if (read(file, buf, 2) != 2) {
        return 0;
    }
    
    // Combine High and Low bytes
    return (int16_t)((buf[0] << 8) | buf[1]);
}

int main() {
    // Open the I2C bus
    if ((file = open(bus, O_RDWR)) < 0) {
        printf("Failed to open the bus.\n");
        return 1;
    }

    init_imu(0x68);
    init_imu(0x69);

    printf("Reading Accel X (Ctrl+C to quit)\n");
    printf("IMU1 (0x68) | IMU2 (0x69)\n");

    while (1) {
        int16_t val1 = read_sensor(0x68, ACCEL_XOUT_H);
        int16_t val2 = read_sensor(0x69, ACCEL_XOUT_H);

        printf("\r%11d | %11d", val1, val2);
        fflush(stdout);
        usleep(100000); // Sleep 100ms
    }

    return 0;
}