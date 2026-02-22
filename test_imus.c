#include <stdio.h>
#include <stdlib.h>
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

// --- Global Setup ---
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

// --- 1. HARDWARE INIT ---
int setup_imu(MPU6050_Device *dev, uint8_t a_range, uint8_t g_range, uint8_t dlpf, uint8_t sample_rate) {
    if (ioctl(file, I2C_SLAVE, dev->addr) < 0) {
        printf("System Error: Failed to connect to I2C bus for IMU at 0x%02x\n", dev->addr);
        return 0;
    }

    int who_am_i = i2c_smbus_read_byte_data(file, WHO_AM_I);
    if (who_am_i != 0x68) {
        printf("Hardware Error: IMU at 0x%02x returned WHO_AM_I = 0x%02x. Check wiring!\n", dev->addr, who_am_i);
        return 0; 
    }
    
    i2c_smbus_write_byte_data(file, PWR_MGMT_1, 0x00); // Wake Up

    // Accelerometer
    i2c_smbus_write_byte_data(file, ACCEL_CONFIG, a_range);
    switch(a_range) {
        case ACCEL_FS_2G:  dev->a_scale = 16384.0; break;
        case ACCEL_FS_4G:  dev->a_scale = 8192.0;  break;
        case ACCEL_FS_8G:  dev->a_scale = 4096.0;  break;
        case ACCEL_FS_16G: dev->a_scale = 2048.0;  break;
        default: dev->a_scale = 16384.0; 
    }

    // Gyroscope
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
    }
}

// --- 3. PRECISION TIMING ---
long get_frame_dt_ns(uint8_t sample_rate_div) {
    long hz = 1000 / (1 + sample_rate_div);
    return 1000000000L / hz;
}

// --- 4. CALIBRATION SYSTEM ---
void ensure_calib_dir_exists(int addr, uint8_t g_range, uint8_t dlpf, int temp_bucket) {
    struct stat st = {0};
    char path[128];
    
    if (stat("calib", &st) == -1) mkdir("calib", 0777);
    
    snprintf(path, sizeof(path), "calib/0x%02x", addr);
    if (stat(path, &st) == -1) mkdir(path, 0777);
    
    snprintf(path, sizeof(path), "calib/0x%02x/G%02x", addr, g_range);
    if (stat(path, &st) == -1) mkdir(path, 0777);

    snprintf(path, sizeof(path), "calib/0x%02x/G%02x/D%02x", addr, g_range, dlpf);
    if (stat(path, &st) == -1) mkdir(path, 0777);

    snprintf(path, sizeof(path), "calib/0x%02x/G%02x/D%02x/%d", addr, g_range, dlpf, temp_bucket);
    if (stat(path, &st) == -1) mkdir(path, 0777);
}

int load_calibration(MPU6050_Device *dev) {
    read_imu_data(dev); // Get current temp
    
    int target_bucket = (int)dev->temp_c; 
    char filepath[128];
    FILE *f = NULL;
    
    int actual_bucket = target_bucket;
    int max_search_radius = 5;
    int found_distance = -1;

    for (int offset = 0; offset <= max_search_radius; offset++) {
        // Try negative
        actual_bucket = target_bucket - offset;
        snprintf(filepath, sizeof(filepath), "calib/0x%02x/G%02x/D%02x/%d/offsets.txt", 
                 dev->addr, dev->g_range, dev->dlpf, actual_bucket);
        f = fopen(filepath, "r");
        if (f) { found_distance = offset; break; }
        
        // Try positive
        if (offset > 0) { 
            actual_bucket = target_bucket + offset;
            snprintf(filepath, sizeof(filepath), "calib/0x%02x/G%02x/D%02x/%d/offsets.txt", 
                     dev->addr, dev->g_range, dev->dlpf, actual_bucket);
            f = fopen(filepath, "r");
            if (f) { found_distance = offset; break; }
        }
    }
    
    if (f) {
        fscanf(f, "%f %f %f %f", &dev->gx_offset, &dev->gy_offset, &dev->gz_offset, &dev->calib_temp);
        fclose(f);
        
        if (found_distance == 0) {
            printf(">> Loaded config for IMU 0x%02x from %s\n", dev->addr, filepath);
        } else if (found_distance <= 5) {
            printf(">> Loaded nearby bucket (%dC) for IMU 0x%02x\n", actual_bucket, dev->addr);
        }
        return 1; 
    }
    
    printf(">> CRITICAL: No calibration found for IMU 0x%02x\n", dev->addr);
    return 0; 
}

void calibrate_gyro_sweep(MPU6050_Device *devs, int num_devs, int num_samples) {
    printf("\n>>> STARTING FULL GYRO CALIBRATION SWEEP (28 Configurations) <<<\n");
    printf("DO NOT MOVE SENSORS. Collecting %d samples per setting at 1000Hz...\n\n", num_samples);
    sleep(3);

    uint8_t g_ranges[] = {GYRO_FS_250, GYRO_FS_500, GYRO_FS_1000, GYRO_FS_2000};
    uint8_t dlpfs[] = {DLPF_260HZ, DLPF_184HZ, DLPF_94HZ, DLPF_44HZ, DLPF_21HZ, DLPF_10HZ, DLPF_5HZ};

    // Arrays to hold memory buffers for parallel processing
    float *gx_log[num_devs], *gy_log[num_devs], *gz_log[num_devs], *temp_log[num_devs];
    
    for (int g = 0; g < 4; g++) {
        for (int d = 0; d < 7; d++) {
            printf("Calibrating -> G_Range: 0x%02x | DLPF: 0x%02x ... ", g_ranges[g], dlpfs[d]);
            fflush(stdout);

            float sum_gx[num_devs], sum_gy[num_devs], sum_gz[num_devs], sum_temp[num_devs];

            // Setup hardware for this iteration (locked to 1000Hz)
            for(int j = 0; j < num_devs; j++) {
                setup_imu(&devs[j], ACCEL_FS_8G, g_ranges[g], dlpfs[d], SAMPLE_1000HZ);
                
                gx_log[j] = (float *)malloc(num_samples * sizeof(float));
                gy_log[j] = (float *)malloc(num_samples * sizeof(float));
                gz_log[j] = (float *)malloc(num_samples * sizeof(float));
                temp_log[j] = (float *)malloc(num_samples * sizeof(float));
                
                sum_gx[j] = 0; sum_gy[j] = 0; sum_gz[j] = 0; sum_temp[j] = 0;
                devs[j].gx_offset = 0.0; devs[j].gy_offset = 0.0; devs[j].gz_offset = 0.0;
            }

            long target_ns = get_frame_dt_ns(SAMPLE_1000HZ);
            struct timespec start, now;
            clock_gettime(CLOCK_MONOTONIC, &start);

            // Precision Collection Loop
            for (int i = 0; i < num_samples; i++) {
                do {
                    clock_gettime(CLOCK_MONOTONIC, &now);
                } while (((now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec)) < target_ns);

                start.tv_nsec += target_ns;
                while (start.tv_nsec >= 1000000000L) {
                    start.tv_nsec -= 1000000000L;
                    start.tv_sec += 1;
                }

                for(int j = 0; j < num_devs; j++) {
                    read_imu_data(&devs[j]);
                    
                    gx_log[j][i] = devs[j].gx_ds;
                    gy_log[j][i] = devs[j].gy_ds;
                    gz_log[j][i] = devs[j].gz_ds;
                    temp_log[j][i] = devs[j].temp_c;

                    sum_gx[j] += devs[j].gx_ds;
                    sum_gy[j] += devs[j].gy_ds;
                    sum_gz[j] += devs[j].gz_ds;
                    sum_temp[j] += devs[j].temp_c;
                }
            }

            // Save Data
            for(int j = 0; j < num_devs; j++) {
                devs[j].gx_offset = sum_gx[j] / num_samples;
                devs[j].gy_offset = sum_gy[j] / num_samples;
                devs[j].gz_offset = sum_gz[j] / num_samples;
                devs[j].calib_temp = sum_temp[j] / num_samples;

                int temp_bucket = (int)devs[j].calib_temp;
                ensure_calib_dir_exists(devs[j].addr, g_ranges[g], dlpfs[d], temp_bucket);

                // Save raw samples
                char csv_path[128];
                snprintf(csv_path, sizeof(csv_path), "calib/0x%02x/G%02x/D%02x/%d/gyro.csv", 
                         devs[j].addr, g_ranges[g], dlpfs[d], temp_bucket);
                FILE *f_csv = fopen(csv_path, "w");
                if (f_csv) {
                    fprintf(f_csv, "Sample,Raw_Gx,Raw_Gy,Raw_Gz,Temp_C\n");
                    for (int i = 0; i < num_samples; i++) {
                        fprintf(f_csv, "%d,%f,%f,%f,%f\n", i, gx_log[j][i], gy_log[j][i], gz_log[j][i], temp_log[j][i]);
                    }
                    fclose(f_csv);
                }

                // Save calculated offsets
                char offset_path[128];
                snprintf(offset_path, sizeof(offset_path), "calib/0x%02x/G%02x/D%02x/%d/offsets.txt", 
                         devs[j].addr, g_ranges[g], dlpfs[d], temp_bucket);
                FILE *f_off = fopen(offset_path, "w");
                if (f_off) {
                    fprintf(f_off, "%f %f %f %f\n", devs[j].gx_offset, devs[j].gy_offset, devs[j].gz_offset, devs[j].calib_temp);
                    fclose(f_off);
                }

                free(gx_log[j]); free(gy_log[j]); free(gz_log[j]); free(temp_log[j]);
            }
            printf("Done.\n");
        }
    }
    printf(">>> FULL GYRO SWEEP COMPLETE <<<\n\n");
}

// --- 5. MAIN ---
int main() {
    file = open(bus, O_RDWR);
    if (file < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    MPU6050_Device imu1 = { .addr = IMU1_ADDR };
    MPU6050_Device imu2 = { .addr = IMU2_ADDR };
    MPU6050_Device my_imus[2] = { imu1, imu2 };

    uint8_t RUN_ACCEL = ACCEL_FS_2G;
    uint8_t RUN_GYRO  = GYRO_FS_500;
    uint8_t RUN_DLPF  = DLPF_94HZ;
    uint8_t RUN_RATE  = SAMPLE_500HZ;

    printf("--- Initializing Hardware ---\n");
    for(int i=0; i<2; i++) {
        if (!setup_imu(&my_imus[i], RUN_ACCEL, RUN_GYRO, RUN_DLPF, RUN_RATE)) return 1;
    }

    printf("\n--- Gyroscope Calibration Check ---\n");
    int imu1_ready = load_calibration(&my_imus[0]);
    int imu2_ready = load_calibration(&my_imus[1]);

    if (!imu1_ready || !imu2_ready) {
        // ---> CHANGED: 500 samples per iteration.
        // At 1000Hz, this is exactly 0.5 seconds of data per setting.
        calibrate_gyro_sweep(my_imus, 2, 500);
        
        // Re-setup target config and load offsets
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