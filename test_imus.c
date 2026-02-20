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

void sweep_dlpf_calibrations(MPU6050_Device *dev) {
    printf("\n>>> STARTING DLPF SWEEP FOR IMU 0x%02x <<<\n", dev->addr);
    printf("DO NOT MOVE SENSOR. This will take ~15 seconds...\n\n");
    sleep(3);

    uint8_t dlpfs[] = {DLPF_260HZ, DLPF_184HZ, DLPF_94HZ, DLPF_44HZ, DLPF_21HZ, DLPF_10HZ, DLPF_5HZ};
    
    // Lock in the run configurations so we don't accidentally sweep ranges
    uint8_t current_a_range = dev->a_range;
    uint8_t current_g_range = dev->g_range;
    uint8_t current_s_rate = dev->sample_rate;

    for(int d = 0; d < 7; d++) {
        setup_imu(dev, current_a_range, current_g_range, dlpfs[d], current_s_rate);
        
        int num_samples = 500;
        float sum_x = 0, sum_y = 0, sum_z = 0, sum_temp = 0;
        
        long target_ns = get_frame_dt_ns(dev->sample_rate);
        struct timespec start, now;
        clock_gettime(CLOCK_MONOTONIC, &start);

        printf("[%d/7] Sweeping DLPF: 0x%02x... ", d+1, dlpfs[d]);
        fflush(stdout);

        for (int i = 0; i < num_samples; i++) {
            do {
                clock_gettime(CLOCK_MONOTONIC, &now);
            } while (((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) < target_ns);

            start.tv_nsec += target_ns;
            while (start.tv_nsec >= 1000000000L) {
                start.tv_nsec -= 1000000000L;
                start.tv_sec += 1;
            }

            read_imu_data(dev);
            sum_x += dev->gx_ds;
            sum_y += dev->gy_ds;
            sum_z += dev->gz_ds;
            sum_temp += dev->temp_c;
        }

        dev->calib_temp = sum_temp / num_samples;
        dev->gx_offset = sum_x / num_samples;
        dev->gy_offset = sum_y / num_samples;
        dev->gz_offset = sum_z / num_samples;

        // 1-degree bucket
        int temp_bucket = (int)dev->calib_temp; 
        ensure_calib_dir_exists(dev->addr, dev->dlpf, temp_bucket);

        char filepath[128];
        snprintf(filepath, sizeof(filepath), "calib/0x%02x/D%02x/%d/offsets.txt", 
                 dev->addr, dev->dlpf, temp_bucket);
                 
        FILE *f = fopen(filepath, "w");
        if (f) {
            fprintf(f, "%f %f %f %f\n", dev->gx_offset, dev->gy_offset, dev->gz_offset, dev->calib_temp);
            fclose(f);
            printf("Done. Temp: %.1f C\n", dev->calib_temp);
        }
    }
    printf(">>> SWEEP COMPLETE FOR 0x%02x <<<\n\n", dev->addr);
}

// --- 5. MAIN LOOP ---
int main() {
    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };

    // Set the configuration you ACTUALLY want to use for the main run
    uint8_t RUN_ACCEL = ACCEL_FS_8G;
    uint8_t RUN_GYRO  = GYRO_FS_1000;
    uint8_t RUN_DLPF  = DLPF_94HZ;
    uint8_t RUN_RATE  = SAMPLE_500HZ;

    printf("--- Initializing Hardware ---\n");
    if (!setup_imu(&imu1, RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE)) return 1;
    if (!setup_imu(&imu2, RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE)) return 1;

    // Check for calibration file. If missing, sweep and reset.
    if (!load_calibration(&imu1)) {
        sweep_dlpf_calibrations(&imu1);
        setup_imu(&imu1, RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE);
        load_calibration(&imu1);
    }
    
    if (!load_calibration(&imu2)) {
        sweep_dlpf_calibrations(&imu2);
        setup_imu(&imu2, RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE); 
        load_calibration(&imu2); 
    }

    printf("\nRunning Sensors at target rate. Press Ctrl+C to stop.\n");
    sleep(1); 
    
    int counter = 0;
    long run_dt_ns = get_frame_dt_ns(RUN_RATE);
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // The jitter-free main loop
    while(1) {
        do {
            clock_gettime(CLOCK_MONOTONIC, &now);
        } while (((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) < run_dt_ns);

        start.tv_nsec += run_dt_ns;
        while (start.tv_nsec >= 1000000000L) {
            start.tv_nsec -= 1000000000L;
            start.tv_sec += 1;
        }

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
    }

    return 0;
}