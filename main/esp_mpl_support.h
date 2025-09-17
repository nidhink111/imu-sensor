/*
 * esp_mpl_support.h
 *
 *  Created on: 13-Sep-2025
 *      Author: nidhi
 */

#ifndef MAIN_ESP_MPL_SUPPORT_H_
#define MAIN_ESP_MPL_SUPPORT_H_

#include "mpu_9250_i2c.h"


static inline int reg_int_cb(struct int_param_s *int_param)
{
		return i2c_master_init();
}

/**
 * MPL I2C read/write functions
 */
#define i2c_write   mpu9250_write_bytes
#define i2c_read    mpu9250_read_bytes

/**
 * MPL delay milliseconds function
 * Gets directly mapped to nordic API (nrf_delay)
 *
 * 				E.g. delay_ms(100) to delay for 100 msec
 */
#define delay_ms    esp_delay_ms

/**
 * MPL function to get current time in milliseconds (nrf51_i2c)
 */
#define get_ms      esp_get_timestamp_ms

///**
// * MPL function to log messages
// * TODO
// */
//#define nrf_log_i(fmt, ...) SEGGER_RTT_printf(0, ##__VA_ARGS__)
//#define nrf_log_e(fmt, ...) SEGGER_RTT_printf(0, ##__VA_ARGS__)

#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)

/**
 * Other definitions
 */
#define min(a,b) ((a<b)?a:b)

#endif /* MAIN_ESP_MPL_SUPPORT_H_ */
