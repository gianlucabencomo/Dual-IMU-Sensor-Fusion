#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include "mpu6050.h"

int file;
const char *bus = "/dev/i2c-1";

typedef struct {
    int addr; 
    float a_scale; 
    float g_scale; 

    // Configuration State
    uint8_t a_range; 
    uint8_t g_range; 
    uint8_t dlpf; 
    uint8_t sample_rate;
    
    // Calibration Offsets & Temp
    float gx_offset, gy_offset, gz_offset;
    float calib_temp;

    // Processed Data
    float ax_g, ay_g, az_g;
    float gx_ds, gy_ds, gz_ds;
    float temp_c;
} MPU6050_Device;

// --- 1. HARDWARE SETUP ---
int setup_imu(MPU6050_Device *dev, uint8_t a_range, uint8_t g_range, uint8_t dlpf, uint8_t sample_rate) {
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) {
        printf("System Error: Failed to connect to I2C bus for IMU at 0x%02x\n", dev->addr);
        return 0;
    }

    int who_am_i = i2c_smbus_read_byte_data(file, WHO_AM_I);
    if (who_am_i != 0x68) {
        printf("Hardware Error: IMU at 0x%02x returned WHO_AM_I = 0x%02x (Expected 0x68). Check wiring!\n", dev->addr, who_am_i);
        return 0; 
    }
    
    i2c_smbus_write_byte_data(file, PWR_MGMT_1, 0x00); // Wake Up
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
    i2c_smbus_write_byte_data(file, SMPLRT_DIV, sample_rate); 

    // Store state
    dev->a_range = a_range;
    dev->g_range = g_range;
    dev->dlpf = dlpf;
    dev->sample_rate = sample_rate;

    // Reset offsets for clean initial reads
    dev->gx_offset = 0; dev->gy_offset = 0; dev->gz_offset = 0;
    return 1;
}

// --- 2. DATA ACQUISITION ---
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

        dev->gx_ds = (raw_gx / dev->g_scale) - dev->gx_offset;
        dev->gy_ds = (raw_gy / dev->g_scale) - dev->gy_offset;
        dev->gz_ds = (raw_gz / dev->g_scale) - dev->gz_offset;
    } else {
        printf("\n[WARNING] I2C Read Failed on IMU 0x%02x!\n", dev->addr);
    }
}

// --- 3. PRECISION TIMING ---
long get_frame_dt_ns(uint8_t sample_rate_div) {
    long hz = 1000 / (1 + sample_rate_div);
    return 1000000000L / hz;
}

// --- 4. CALIBRATION & DIRECTORIES ---
void ensure_calib_dir_exists(int addr, uint8_t dlpf, int temp_bucket) {
    struct stat st = {0};
    char path[128];
    
    if (stat("calib", &st) == -1) mkdir("calib", 0777);
    
    snprintf(path, sizeof(path), "calib/0x%02x", addr);
    if (stat(path, &st) == -1) mkdir(path, 0777);
    
    snprintf(path, sizeof(path), "calib/0x%02x/D%02x", addr, dlpf);
    if (stat(path, &st) == -1) mkdir(path, 0777);

    snprintf(path, sizeof(path), "calib/0x%02x/D%02x/%d", addr, dlpf, temp_bucket);
    if (stat(path, &st) == -1) mkdir(path, 0777);
}

int load_calibration(MPU6050_Device *dev) {
    // Take one quick reading to get current hardware temperature
    read_imu_data(dev); 
    
    // 1-degree buckets
    int target_bucket = (int)dev->temp_c; 
    char filepath[128];
    FILE *f = NULL;
    
    int actual_bucket = target_bucket;
    int max_search_radius = 15; 
    int found_distance = -1;

    // Expanding search pattern: 0, -1, +1, -2, +2...
    for (int offset = 0; offset <= max_search_radius; offset++) {
        
        // Try negative offset first
        actual_bucket = target_bucket - offset;
        snprintf(filepath, sizeof(filepath), "calib/0x%02x/D%02x/%d/offsets.txt", 
                 dev->addr, dev->dlpf, actual_bucket);
        f = fopen(filepath, "r");
        if (f) {
            found_distance = offset;
            break;
        }
        
        // Try positive offset
        if (offset > 0) { 
            actual_bucket = target_bucket + offset;
            snprintf(filepath, sizeof(filepath), "calib/0x%02x/D%02x/%d/offsets.txt", 
                     dev->addr, dev->dlpf, actual_bucket);
            f = fopen(filepath, "r");
            if (f) {
                found_distance = offset;
                break;
            }
        }
    }
    
    // Evaluate what we found
    if (f) {
        fscanf(f, "%f %f %f %f", &dev->gx_offset, &dev->gy_offset, &dev->gz_offset, &dev->calib_temp);
        fclose(f);
        
        if (found_distance == 0) {
            printf(">> Loaded exact config for IMU 0x%02x from %s\n", dev->addr, filepath);
        } else if (found_distance <= 5) {
            printf(">> Note: Temp %dC missing. Loaded nearby bucket (%dC) for IMU 0x%02x\n", 
                   target_bucket, actual_bucket, dev->addr);
        } else {
            printf("\n[WARNING] IMU 0x%02x calibration is %d degrees off! (Target: %dC, Found: %dC).\n", 
                   dev->addr, found_distance, target_bucket, actual_bucket);
            printf("          Thermal drift may cause gyro errors. Recalibration recommended.\n\n");
        }
        return 1; // Success
    }
    
    printf(">> CRITICAL: No calibration found within %d degrees of %dC for IMU 0x%02x\n", 
           max_search_radius, target_bucket, dev->addr);
    return 0; // File not found
}

void sweep_dual_calibrations(MPU6050_Device *devs, int num_devs) {
    printf("\n>>> STARTING SIMULTANEOUS DLPF SWEEP FOR %d IMUs <<<\n", num_devs);
    printf("DO NOT MOVE SENSORS. Capturing thermal snapshots...\n\n");
    sleep(3);

    uint8_t dlpfs[] = {DLPF_260HZ, DLPF_184HZ, DLPF_94HZ, DLPF_44HZ, DLPF_21HZ, DLPF_10HZ, DLPF_5HZ};
    
    // Use the settings from the first device as the template for the sweep
    uint8_t a_r = devs[0].a_range;
    uint8_t g_r = devs[0].g_range;
    uint8_t s_r = devs[0].sample_rate;

    for(int d = 0; d < 7; d++) {
        // 1. Setup all devices for this DLPF
        for(int j = 0; j < num_devs; j++) {
            setup_imu(&devs[j], a_r, g_r, dlpfs[d], s_r);
        }

        int num_samples = 500;
        float sum_x[num_devs], sum_y[num_devs], sum_z[num_devs], sum_temp[num_devs];
        
        // Initialize sums
        for(int j=0; j<num_devs; j++) {
            sum_x[j] = sum_y[j] = sum_z[j] = sum_temp[j] = 0;
            devs[j].gx_offset = devs[j].gy_offset = devs[j].gz_offset = 0; // Clear offsets for raw read
        }

        long target_ns = get_frame_dt_ns(s_r);
        struct timespec start, now;
        clock_gettime(CLOCK_MONOTONIC, &start);

        printf("[%d/7] Sweeping DLPF: 0x%02x... ", d+1, dlpfs[d]);
        fflush(stdout);

        // 2. Parallel Precision Collection
        for (int i = 0; i < num_samples; i++) {
            do {
                clock_gettime(CLOCK_MONOTONIC, &now);
            } while (((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) < target_ns);

            start.tv_nsec += target_ns;
            while (start.tv_nsec >= 1000000000L) {
                start.tv_nsec -= 1000000000L;
                start.tv_sec += 1;
            }

            // Read from all devices in the same time slice
            for(int j=0; j < num_devs; j++) {
                read_imu_data(&devs[j]);
                sum_x[j] += devs[j].gx_ds;
                sum_y[j] += devs[j].gy_ds;
                sum_z[j] += devs[j].gz_ds;
                sum_temp[j] += devs[j].temp_c;
            }
        }

        // 3. Save results for all devices
        for(int j=0; j < num_devs; j++) {
            devs[j].calib_temp = sum_temp[j] / num_samples;
            devs[j].gx_offset = sum_x[j] / num_samples;
            devs[j].gy_offset = sum_y[j] / num_samples;
            devs[j].gz_offset = sum_z[j] / num_samples;

            int temp_bucket = (int)devs[j].calib_temp; 
            ensure_calib_dir_exists(devs[j].addr, dlpfs[d], temp_bucket);

            char filepath[128];
            snprintf(filepath, sizeof(filepath), "calib/0x%02x/D%02x/%d/offsets.txt", 
                     devs[j].addr, dlpfs[d], temp_bucket);
                     
            FILE *f = fopen(filepath, "w");
            if (f) {
                fprintf(f, "%f %f %f %f\n", devs[j].gx_offset, devs[j].gy_offset, devs[j].gz_offset, devs[j].calib_temp);
                fclose(f);
            }
        }
        printf("Done.\n");
    }
    printf(">>> DUAL SWEEP COMPLETE <<<\n\n");
}

// --- 5. UPDATED MAIN ---
int main() {
    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };
    MPU6050_Device my_imus[2] = { imu1, imu2 };

    uint8_t RUN_ACCEL = ACCEL_FS_8G;
    uint8_t RUN_GYRO  = GYRO_FS_1000;
    uint8_t RUN_DLPF  = DLPF_94HZ;
    uint8_t RUN_RATE  = SAMPLE_500HZ;

    printf("--- Initializing Hardware ---\n");
    for(int i=0; i<2; i++) {
        if (!setup_imu(&my_imus[i], RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE)) return 1;
    }

    printf("\n--- Gyroscope Calibration Check ---\n");
    int imu1_ready = load_calibration(&my_imus[0]);
    int imu2_ready = load_calibration(&my_imus[1]);

    // If EITHER imu is missing calibration, run the sweep for both
    if (!imu1_ready || !imu2_ready) {
        sweep_dual_calibrations(my_imus, 2);
        // Re-setup and reload after sweep
        for(int i=0; i<2; i++) {
            setup_imu(&my_imus[i], RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE);
            load_calibration(&my_imus[i]);
        }
    }

    printf("\nRunning Sensors at target rate. Press Ctrl+C to stop.\n");
    sleep(1); 
    
    int counter = 0;
    long run_dt_ns = get_frame_dt_ns(RUN_RATE);
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while(1) {
        do {
            clock_gettime(CLOCK_MONOTONIC, &now);
        } while (((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) < run_dt_ns);

        start.tv_nsec += run_dt_ns;
        while (start.tv_nsec >= 1000000000L) {
            start.tv_nsec -= 1000000000L;
            start.tv_sec += 1;
        }

        read_imu_data(&my_imus[0]);
        read_imu_data(&my_imus[1]);

        if (counter % 50 == 0) {
            printf("\033[2J\033[H"); 
            printf("=== IMU 1 (0x68) === [Temp: %.1f C]\n", my_imus[0].temp_c);
            printf("ACCEL: X:%6.2f Y:%6.2f Z:%6.2f\n", my_imus[0].ax_g, my_imus[0].ay_g, my_imus[0].az_g);
            printf("GYRO:  X:%6.1f Y:%6.1f Z:%6.1f\n", my_imus[0].gx_ds, my_imus[0].gy_ds, my_imus[0].gz_ds);
            
            printf("\n=== IMU 2 (0x69) === [Temp: %.1f C]\n", my_imus[1].temp_c);
            printf("ACCEL: X:%6.2f Y:%6.2f Z:%6.2f\n", my_imus[1].ax_g, my_imus[1].ay_g, my_imus[1].az_g);
            printf("GYRO:  X:%6.1f Y:%6.1f Z:%6.1f\n", my_imus[1].gx_ds, my_imus[1].gy_ds, my_imus[1].gz_ds);
        }
        counter++;
    }
    return 0;
}