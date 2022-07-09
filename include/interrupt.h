// Library headers
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"


// Project headers


#define WAITresetPIN 33
void WAITreset(void);


void IRAM_ATTR myIRQ(void *arg);
void configGPIO_IRQ(void);
