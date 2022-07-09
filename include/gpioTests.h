// Library headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"


void task_directREG(void *ignore);
void task_tick_gpio(void *ignore);
void task_tick_iomux(void *ignore);
void task_tick_rtc(void *ignore);