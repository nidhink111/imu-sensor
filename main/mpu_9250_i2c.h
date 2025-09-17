/*
 * mpu_9250_i2c.h
 *
 *  Created on: 13-Sep-2025
 *      Author: nidhi
 */

#ifndef MAIN_MPU_9250_I2C_H_
#define MAIN_MPU_9250_I2C_H_



#include "esp_err.h"

void imu_task_start(void);
int  i2c_master_init(void);
esp_err_t mpu9250_write_bytes(unsigned char slave_addr ,uint8_t reg_addr, size_t len, const uint8_t *data);
esp_err_t mpu9250_read_bytes(unsigned char slave_addr,uint8_t reg_addr, size_t len, uint8_t *data);
void esp_delay_ms(uint32_t delay_ms);
uint64_t esp_get_timestamp_ms(void);


#endif /* MAIN_MPU_9250_I2C_H_ */
