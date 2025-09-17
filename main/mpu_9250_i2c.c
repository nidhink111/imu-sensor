/*
 * mpu_9250_i2c.c
 *
 *  Created on: 13-Sep-2025
 *      Author: nidhi
 */


#include <stdio.h>
#include <string.h> 
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "algo/param.h"


// I2C Configuration
// I2C Configuration
#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000

// MPU9250 Addresses
#define MPU9250_ADDRESS            0x68
#define AK8963_ADDRESS             0x0C

// MPU9250 Register Map
#define WHO_AM_I_MPU9250           0x75
#define PWR_MGMT_1                 0x6B
#define ACCEL_CONFIG               0x1C
#define GYRO_CONFIG                0x1B
#define ACCEL_XOUT_H               0x3B
#define GYRO_XOUT_H                0x43
#define TEMP_OUT_H                 0x41
#define CONFIG                     0x1A
#define SMPLRT_DIV                 0x19

// Accelerometer full scale ranges
#define AFS_2G                     0x00
#define AFS_4G                     0x08
#define AFS_8G                     0x10
#define AFS_16G                    0x18

// Gyroscope full scale ranges
#define GFS_250DPS                 0x00
#define GFS_500DPS                 0x08
#define GFS_1000DPS                0x10
#define GFS_2000DPS                0x18

// Sensitivity scales (LSB per unit)
#define ACCEL_SCALE_2G             16384.0f
#define ACCEL_SCALE_4G             8192.0f
#define ACCEL_SCALE_8G             4096.0f
#define ACCEL_SCALE_16G            2048.0f

#define GYRO_SCALE_250DPS          131.0f
#define GYRO_SCALE_500DPS          65.5f
#define GYRO_SCALE_1000DPS         32.8f
#define GYRO_SCALE_2000DPS         16.4f

#define AK8963_WHO_AM_I   0x00
#define AK8963_ST1        0x02
#define AK8963_HXL        0x03
#define AK8963_CNTL1      0x0A
#define AK8963_CNTL2      0x0B

#define AK8963_FUSE_ROM   0x10
#define AK8963_MODE_POWER_DOWN  0x00
#define AK8963_MODE_FUSE_ROM   0x0F
#define AK8963_MODE_CONTINUOUS 0x16  // 16-bit output, continuous mode

static const char *TAG = "MPU9250";

// I2C device handles
static i2c_master_dev_handle_t mpu9250_dev_handle = NULL;
static i2c_master_dev_handle_t ak8963_dev_handle = NULL;

// Sensor configuration
typedef struct {
    uint8_t accel_fs;
    uint8_t gyro_fs;
    float accel_scale;
    float gyro_scale;
} mpu9250_config_t;

// Sensor data structure
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temp_c;
    
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    float mag_x_uT;
    float mag_y_uT;
    float mag_z_uT;
} mpu9250_data_t;

static mpu9250_config_t mpu_config;


void esp_delay_ms(uint32_t delay_ms)
{
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}
uint64_t esp_get_timestamp_ms(void)
{
    // esp_timer_get_time() returns microseconds since boot
    return esp_timer_get_time() / 1000ULL;
}
// I2C Master Initialization
int  i2c_master_init(void) {
     ESP_LOGI(TAG, "Initializing I2C master with new driver...");

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    // Configure MPU9250 device
    i2c_device_config_t mpu9250_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu9250_dev_cfg, &mpu9250_dev_handle));

    // Configure AK8963 device (magnetometer)
    i2c_device_config_t ak8963_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AK8963_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ak8963_dev_cfg, &ak8963_dev_handle));

    return ESP_OK;

}

// Write byte to I2C device
// Write byte to I2C device using new driver
esp_err_t mpu9250_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(mpu9250_dev_handle, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

// Write multiple bytes to I2C device
esp_err_t mpu9250_write_bytes(unsigned char slave_addr ,uint8_t reg_addr, size_t len, const uint8_t *data) {
    uint8_t *write_buf = malloc(len + 1);
    if (write_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    write_buf[0] = reg_addr;
    memcpy(write_buf + 1, data, len);
    
    esp_err_t ret = i2c_master_transmit(mpu9250_dev_handle, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);
    
    free(write_buf);
    return ret;
}

// Read byte from I2C device
esp_err_t mpu9250_read_byte(uint8_t reg_addr, uint8_t *data) {
    // First write the register address
    esp_err_t ret = i2c_master_transmit(mpu9250_dev_handle, &reg_addr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Then read the data
    return i2c_master_receive(mpu9250_dev_handle, data, 1, 1000 / portTICK_PERIOD_MS);
}

// Read multiple bytes from I2C device
esp_err_t mpu9250_read_bytes(unsigned char slave_addr,uint8_t reg_addr, size_t len, uint8_t *data) {
    // First write the register address
    esp_err_t ret = i2c_master_transmit(mpu9250_dev_handle, &reg_addr, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Then read the data
    return i2c_master_receive(mpu9250_dev_handle, data, len, 1000 / portTICK_PERIOD_MS);
}

// Set gyroscope full scale range
esp_err_t mpu9250_set_gyro_fs(uint8_t fs_range) {
    ESP_LOGI(TAG, "Setting gyroscope range: 0x%02X", fs_range);
    esp_err_t ret = mpu9250_write_byte(GYRO_CONFIG, fs_range);
    
    if (ret == ESP_OK) {
        switch (fs_range) {
            case GFS_250DPS:
                mpu_config.gyro_scale = GYRO_SCALE_250DPS;
                break;
            case GFS_500DPS:
                mpu_config.gyro_scale = GYRO_SCALE_500DPS;
                break;
            case GFS_1000DPS:
                mpu_config.gyro_scale = GYRO_SCALE_1000DPS;
                break;
            case GFS_2000DPS:
                mpu_config.gyro_scale = GYRO_SCALE_2000DPS;
                break;
            default:
                mpu_config.gyro_scale = GYRO_SCALE_250DPS;
                break;
        }
        mpu_config.gyro_fs = fs_range;
    }
    
    return ret;
}
// Set accelerometer full scale range
esp_err_t mpu9250_set_accel_fs(uint8_t fs_range) {
    ESP_LOGI(TAG, "Setting accelerometer range: 0x%02X", fs_range);
    esp_err_t ret = mpu9250_write_byte(ACCEL_CONFIG, fs_range);
    
    if (ret == ESP_OK) {
        switch (fs_range) {
            case AFS_2G:
                mpu_config.accel_scale = ACCEL_SCALE_2G;
                break;
            case AFS_4G:
                mpu_config.accel_scale = ACCEL_SCALE_4G;
                break;
            case AFS_8G:
                mpu_config.accel_scale = ACCEL_SCALE_8G;
                break;
            case AFS_16G:
                mpu_config.accel_scale = ACCEL_SCALE_16G;
                break;
            default:
                mpu_config.accel_scale = ACCEL_SCALE_2G;
                break;
        }
        mpu_config.accel_fs = fs_range;
    }
    
    return ret;
}
// Initialize MPU9250 with custom ranges
esp_err_t mpu9250_init(uint8_t accel_range, uint8_t gyro_range) {
    // Wake up device
    ESP_ERROR_CHECK(mpu9250_write_byte(PWR_MGMT_1, 0x00));
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Check WHO_AM_I
    uint8_t whoami;
    ESP_ERROR_CHECK(mpu9250_read_byte(WHO_AM_I_MPU9250, &whoami));
    ESP_LOGI(TAG, "MPU9250 WHO_AM_I: 0x%02X", whoami);

    if (whoami != 0x71) {
        ESP_LOGE(TAG, "MPU9250 not found!");
        return ESP_FAIL;
    }

    // Configure digital low-pass filter (DLPF)
    ESP_ERROR_CHECK(mpu9250_write_byte(CONFIG, 0x03));
    
    // Set sample rate divider (1kHz / (1 + 4) = 200Hz)
    ESP_ERROR_CHECK(mpu9250_write_byte(SMPLRT_DIV, 0x04));

    // Configure accelerometer range
    ESP_ERROR_CHECK(mpu9250_set_accel_fs(accel_range));
    
    // Configure gyroscope range
    ESP_ERROR_CHECK(mpu9250_set_gyro_fs(gyro_range));

    // Reset AK8963
	uint8_t data;
	// Enable bypass to access AK8963 directly
mpu9250_write_byte(0x37, 0x02); // INT_PIN_CFG: BYPASS_EN=1
vTaskDelay(500 / portTICK_PERIOD_MS);

mpu9250_write_byte(AK8963_CNTL1, AK8963_MODE_CONTINUOUS);
vTaskDelay(pdMS_TO_TICKS(20));

printf("Mag\n");
// Put magnetometer in power-down before changing modes
i2c_master_transmit(ak8963_dev_handle, (uint8_t[]){AK8963_CNTL1}, 1, 100/portTICK_PERIOD_MS);
uint8_t cntl = AK8963_MODE_POWER_DOWN;
i2c_master_transmit(ak8963_dev_handle, (uint8_t[]){AK8963_CNTL1, cntl}, 2, 100/portTICK_PERIOD_MS);
vTaskDelay(pdMS_TO_TICKS(20));
// Check AK8963 WHO_AM_I
uint8_t ak_whoami = 0;
i2c_master_transmit(ak8963_dev_handle, (uint8_t[]){0x00}, 1, 1000/portTICK_PERIOD_MS); // WHO_AM_I = 0x00
i2c_master_receive(ak8963_dev_handle, &ak_whoami, 1, 1000/portTICK_PERIOD_MS);
ESP_LOGI(TAG, "AK8963 WHO_AM_I: 0x%02X", ak_whoami);  // expect 0x48

	mpu9250_write_byte(AK8963_CNTL2, 0x01);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
	// Power down, then enter fuse ROM access mode
	mpu9250_write_byte(AK8963_CNTL1, AK8963_MODE_POWER_DOWN);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	mpu9250_write_byte(AK8963_CNTL1, AK8963_MODE_FUSE_ROM);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	
	// Read factory sensitivity adjustment (optional for calibration)
	
	// Power down again, then start continuous mode
	mpu9250_write_byte(AK8963_CNTL1, AK8963_MODE_POWER_DOWN);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	mpu9250_write_byte(AK8963_CNTL1, AK8963_MODE_CONTINUOUS);
	
	vTaskDelay(10 / portTICK_PERIOD_MS);


    return ESP_OK;
}
esp_err_t ak8963_read_data(mpu9250_data_t *data) {
    uint8_t st1;
    esp_err_t ret = i2c_master_transmit(ak8963_dev_handle, (uint8_t[]){AK8963_ST1}, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    ret = i2c_master_receive(ak8963_dev_handle, &st1, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK || !(st1 & 0x01)) return ESP_FAIL; // no new data

    uint8_t raw[6];
    ret = i2c_master_transmit(ak8963_dev_handle, (uint8_t[]){AK8963_HXL}, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    ret = i2c_master_receive(ak8963_dev_handle, raw, 6, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    data->mag_x = (raw[1] << 8) | raw[0];
    data->mag_y = (raw[3] << 8) | raw[2];
    data->mag_z = (raw[5] << 8) | raw[4];

    // Convert to µT (AK8963 default scale factor = 0.15 µT/LSB for 16-bit)
    data->mag_x_uT = data->mag_x * 0.15f;
    data->mag_y_uT = data->mag_y * 0.15f;
    data->mag_z_uT = data->mag_z * 0.15f;

    return ESP_OK;
}

// Read and convert sensor data
esp_err_t mpu9250_read_data(mpu9250_data_t *data) {
    uint8_t buffer[14];
    
    // Read accelerometer, temperature, and gyroscope
    ESP_ERROR_CHECK(mpu9250_read_bytes(MPU9250_ADDRESS,ACCEL_XOUT_H, 14, buffer));
    
    // Raw data
    data->accel_x = (buffer[0] << 8) | buffer[1];
    data->accel_y = (buffer[2] << 8) | buffer[3];
    data->accel_z = (buffer[4] << 8) | buffer[5];
    data->temp = (buffer[6] << 8) | buffer[7];
    data->gyro_x = (buffer[8] << 8) | buffer[9];
    data->gyro_y = (buffer[10] << 8) | buffer[11];
    data->gyro_z = (buffer[12] << 8) | buffer[13];

    // Convert to physical units
    data->accel_x_g = (float)data->accel_x / mpu_config.accel_scale;
    data->accel_y_g = (float)data->accel_y / mpu_config.accel_scale;
    data->accel_z_g = (float)data->accel_z / mpu_config.accel_scale;
    
    data->gyro_x_dps = (float)data->gyro_x / mpu_config.gyro_scale;
    data->gyro_y_dps = (float)data->gyro_y / mpu_config.gyro_scale;
    data->gyro_z_dps = (float)data->gyro_z / mpu_config.gyro_scale;
    
    data->temp_c = ((float)data->temp / 340.0) + 36.53;

    return ESP_OK;
}


void imu_task_start(void) {
	imu_t a,g,m;
   // Initialize I2C with new driver
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully with new master driver");

    // Initialize MPU9250 with 16g accelerometer and 2000dps gyroscope
    if (mpu9250_init(AFS_16G, GFS_2000DPS) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250");
        return;
    }

    ESP_LOGI(TAG, "MPU9250 initialized with 16g accelerometer and 2000dps gyroscope");

    mpu9250_data_t sensor_data;
    uint32_t counter = 0;

    while (1) {
        if (mpu9250_read_data(&sensor_data) == ESP_OK ) {
            printf("\n=== MPU9250 Sensor Data [%lu] ===\n", counter++);
            printf("Accelerometer (16g range):\n");
            printf("  X: %6.2f g, Raw: %6d\n", sensor_data.accel_x_g, sensor_data.accel_x);
            printf("  Y: %6.2f g, Raw: %6d\n", sensor_data.accel_y_g, sensor_data.accel_y);
            printf("  Z: %6.2f g, Raw: %6d\n", sensor_data.accel_z_g, sensor_data.accel_z);
            
            printf("Gyroscope (2000dps range):\n");
            printf("  X: %7.1f °/s, Raw: %6d\n", sensor_data.gyro_x_dps, sensor_data.gyro_x);
            printf("  Y: %7.1f °/s, Raw: %6d\n", sensor_data.gyro_y_dps, sensor_data.gyro_y);
            printf("  Z: %7.1f °/s, Raw: %6d\n", sensor_data.gyro_z_dps, sensor_data.gyro_z);
            
            printf("Temperature: %.2f °C, Raw: %6d\n", sensor_data.temp_c, sensor_data.temp);
            printf("============================\n");

        } else {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }
        if( ak8963_read_data(&sensor_data) == ESP_OK)
        {
			printf("Magnetometer:\n");
		    printf("  X: %.2f µT, Raw: %d\n", sensor_data.mag_x_uT, sensor_data.mag_x);
		    printf("  Y: %.2f µT, Raw: %d\n", sensor_data.mag_y_uT, sensor_data.mag_y);
		    printf("  Z: %.2f µT, Raw: %d\n", sensor_data.mag_z_uT, sensor_data.mag_z);
		}else {
            ESP_LOGE(TAG, "Failed to read Magnetometer data");
        }

		a.x = sensor_data.accel_x;
		a.y = sensor_data.accel_y;
		a.z = sensor_data.accel_x;

		g.x = sensor_data.gyro_x;
		g.y = sensor_data.gyro_y;
		g.z = sensor_data.gyro_z;
			
	    m.x = sensor_data.mag_x_uT;
	    m.y = sensor_data.mag_y_uT;
	    m.z = sensor_data.mag_z_uT;
    
 		update_quaternions(quaternions,a.x,a.y,a.z,g.x*DEGPERSEC*DEGTORAD,g.y*DEGPERSEC*DEGTORAD,g.z*DEGPERSEC*DEGTORAD,m.x,m.y,m.z,DT,/*BETA*/beta);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}