#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu_9250_i2c.h"
#include "algo/param.h"


void app_main(void)
{

 imu_task_start(); 
 
}
