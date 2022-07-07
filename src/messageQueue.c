#include "messageQueue.h"

//
//  xQueue Messaging system
//

/*
static QueueHandle_t messageQueue;
#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(AMessage)
typedef struct A_Message
{
    uint32_t ucMessageID;
    char ucData[20];
} AMessage;
*/

void sendMessage(int32_t ID, char *message)
{
    AMessage testMessage;
    testMessage.ucMessageID = ID;
    strncpy(testMessage.ucData, message, 20);
    xQueueSend(messageQueue, (void *)&testMessage, 1);
}

void sendAddress(int32_t ID, uint16_t address)
{
    char chrAddr[6];
    itoa(address, chrAddr, 10);

    AMessage testMessage;
    testMessage.ucMessageID = ID;
    strcpy(testMessage.ucData, chrAddr);
    strcat(testMessage.ucData, "\n");
    xQueueSend(messageQueue, &testMessage, 10);
}

//
//  xQueue Message to Serial Port
//

#define ECHO_UART_PORT_NUM (UART_NUM_0)
//#define ECHO_UART_BAUD_RATE (115200)
#define ECHO_UART_BAUD_RATE (230400)
#define ECHO_TASK_STACK_SIZE (2048)
#define BUF_SIZE (1024)
static void task_SerialMessageReporter(void *arg)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));

    vTaskDelay(10);

    // Queue message sender
    AMessage item;
    char charID[10];

    while (1)
    {
        // uart_tx_chars(ECHO_UART_PORT_NUM, "Top\n", sizeof(char) * strnlen("Top\n", 20));

        while (uxQueueMessagesWaiting(messageQueue) > 0)
        {
            if (xQueueReceive(messageQueue, &item, 10))
            {
                uart_tx_chars(ECHO_UART_PORT_NUM, "MSG: ", sizeof(char) * strnlen("MSG: ", 20));
                uart_tx_chars(ECHO_UART_PORT_NUM, itoa(item.ucMessageID, charID, 10), sizeof(char) * strnlen(itoa(item.ucMessageID, charID, 10), 20));
                uart_tx_chars(ECHO_UART_PORT_NUM, " ", sizeof(char) * strnlen(" ", 20));
                uart_tx_chars(ECHO_UART_PORT_NUM, item.ucData, sizeof(char) * strnlen(item.ucData, 20));
            }
        }
        // else
        {
            // uart_tx_chars(ECHO_UART_PORT_NUM, "NADA\n", sizeof(char) * strnlen("NADA\n", 20));
            sendMessage(1, "NADA\n");
            vTaskDelay(50);
        }
    }
}

void startMessageQueue(void)
{
    messageQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    xTaskCreate(task_SerialMessageReporter, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}