#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "driver/spi_master.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"

static QueueHandle_t messageQueue;
#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(AMessage)
typedef struct A_Message
{
    uint32_t ucMessageID;
    char ucData[20];
} AMessage;

void sendMessage(int32_t ID, char *message);
void sendAddress(int32_t ID, uint16_t address);

static void task_SerialMessageReporter(void *arg);

void startMessageQueue(void);