

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

#include "hal/gpio_hal.h"

//
//  GPIO testing (Tasks)
//

void task_directREG(void *ignore)
{
    // This needs a proper BIT mask

    gpio_set_direction(33, GPIO_MODE_OUTPUT);
    while (1)
    {
        // 125uS -- ATMel is still quicker LoLz
        // REG_CLR_BIT(GPIO_OUT1_REG, 0xFF);
        // REG_SET_BIT(GPIO_OUT1_REG, 0xFF);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void task_tick_gpio(void *ignore)
{

    gpio_set_direction(33, GPIO_MODE_OUTPUT);
    while (1)
    {
        // 416.6nS -- Best so far and its still slower than ATMega 125nS
        gpio_set_level(33, 0);
        gpio_set_level(33, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void task_tick_iomux(void *ignore)
{

    gpio_set_direction(33, GPIO_MODE_OUTPUT);

    while (1)
    {
        // 708.3nS -- and they say this is for "high speed digital functions" LoLz
        gpio_iomux_out(33, FUNC_GPIO33_GPIO33_0, 0);
        gpio_iomux_out(33, FUNC_GPIO33_GPIO33_0, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void task_tick_rtc(void *ignore)
{
    rtc_gpio_init(33);
    rtc_gpio_set_direction(33, RTC_GPIO_MODE_OUTPUT_ONLY);

    while ((1))
    {
        // 2.04uS
        rtc_gpio_set_level(33, 0x00000000);
        rtc_gpio_set_level(33, 0xffffffff);

        // This could have been a dirty trick, but it doesn't work.
        /*
        rtc_gpio_pullup_dis(33);
        rtc_gpio_pulldown_en(33);
        rtc_gpio_pulldown_dis(33);
        rtc_gpio_pullup_en(33);
        */

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

//
//  Z80 WAIT signal
//

#define WAITresetPIN 33
void WAITreset()
{

    // GPIO_OUT1_REG - BIT2 = GPIO33
    REG_CLR_BIT(GPIO_OUT1_REG, 2);
    REG_SET_BIT(GPIO_OUT1_REG, 2);

    // gpio_set_level(WAITresetPIN, 0);
    // gpio_set_level(WAITresetPIN, 1);
}

//
//  xQueue Messaging system
//

static QueueHandle_t messageQueue;
#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(AMessage)
typedef struct A_Message
{
    uint32_t ucMessageID;
    char ucData[20];
} AMessage;

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

//
//  Interrupt Service
//

#define IRQPIN 32
void IRAM_ATTR myIRQ(void *arg)
{
    AMessage testMessage;
    testMessage.ucMessageID = IRQPIN;
    strcpy(testMessage.ucData, "IRQ\n");
    xQueueSendFromISR(messageQueue, &testMessage, NULL);

    WAITreset();

    // xQueueOverwrite(messageQueue,&testMessage)
    // xQueueOverwriteFromISR(messageQueue, &testMessage,NULL);
}

void configGPIO_IRQ(void)
{
    // Install driver service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // configure IO interupt
    uint8_t pin = IRQPIN;

    gpio_config_t io_conf;
    io_conf.pin_bit_mask = 1ULL << pin;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // gpio_pullup_en(IRQPIN);
    // gpio_set_direction(IRQPIN, GPIO_MODE_INPUT);
    // gpio_intr_enable(IRQPIN);

    int result = gpio_isr_handler_add((gpio_num_t)IRQPIN, myIRQ, (void *)&pin);

    if (result == ESP_OK)
    {
        sendMessage(0, "ESP_OK\n");
    }
    if (result == ESP_ERR_INVALID_STATE)
    {
        sendMessage(0, "ESP_ERR_INVALID_STATE\n");
    }
    if (result == ESP_ERR_INVALID_ARG)
    {
        sendMessage(0, "ESP_ERR_INVALID_ARG\n");
    }
}

//
//  SPI S-Reg Input
//
spi_host_device_t spiHost_device = HSPI_HOST;
spi_device_handle_t z80handle;
spi_transaction_t z80ReadBusTransaction;

//
// SPI Bus Config
//
const spi_bus_config_t mySPIBus_config = {
    .mosi_io_num = GPIO_NUM_23, // Master Out Slave In GPIO (-1 not used)
    .miso_io_num = GPIO_NUM_18, // Master In Slave Out (-1 not used)
    .sclk_io_num = GPIO_NUM_19, // Spi CLocK signal
    .quadwp_io_num = -1,        // WP (Write Protect) D2 4-bit mode (-1 not used)
    .quadhd_io_num = -1,        // HD (HolD) D3 in 4-bit mode (-1 not used)
    .max_transfer_sz = 24,      // Max transfer in bytes. Defaults to 4094 if 0.
    //.flags = 0,            // Abilities of bus to be checked by the driver
    //.intr_flags = 0,       // Interrupt flag for the bus to set the priority, and IRAM attribute
};

//
// SPI Device Config
//
const spi_device_interface_config_t z80device_config = {
    .command_bits = 8,  // Command phase bits
    .address_bits = 16, // Address phase bits
    .dummy_bits = 0,    // Padding bits for delay before data bits
    .mode = 0,          // SPI mode (CPOL,CPHA?)
    //.duty_cycle_pos = 0,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    //.cs_ena_pretrans = 0,  // SPI bit-cycle the CS is active BEFORE Transmission
    //.cs_ena_posttrans = 0, // SPI bit-cycle the CS is active AFTER Transmission

    .clock_speed_hz = SPI_MASTER_FREQ_8M, // DEBUG - logic analyser 24Mhz max
    // z80dev.clock_speed_hz = SPI_MASTER_FREQ_80M; // Target speed

    //.input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    .spics_io_num = -1, // GPIO pin for ChipSelect (-1 not used)
    //.flags = 0,          // SPI_DEVICE_ * flags?
    .queue_size = 1, // Transactions that can be in flight

    //.pre_cb = NULL,  // IRAM callback BEFORE Transmission
    //.post_cb = NULL, // IRAM callback AFTER Transmission
};

static uint8_t spiBufferTX[4];
static uint8_t spiBufferRX[4];

void config_SPIZ80(void)
{

    //
    // Transaction struct -- Z80 RECEIVE BUS
    //
    z80ReadBusTransaction.flags = 0;
    z80ReadBusTransaction.cmd = 8;      // Command bits
    z80ReadBusTransaction.addr = 16;    // Addr bits
    z80ReadBusTransaction.length = 8;   // Total Data bits
    z80ReadBusTransaction.rxlength = 8; // Total Data Received in bits (less than total data bits) [0 = length]
    // z80ReadBusTransaction.user = NULL;  // Void pointer for "User defined" E.G TransactionID

    z80ReadBusTransaction.tx_buffer = &spiBufferTX;
    // z80transaction.tx_data=NULL; // < If SPI_TRANS_USE_TXDATA is set, data set here is sent directly from this variable.

    z80ReadBusTransaction.rx_buffer = &spiBufferRX;
    // z80transaction.rx_data=NULL;  // < If SPI_TRANS_USE_RXDATA is set, data is received directly to this variable

    //
    // INIT
    //
    uint8_t result;
    result = spi_bus_initialize(spiHost_device, &mySPIBus_config, SPI_DMA_CH_AUTO);
    sendMessage(result, "spi_bus_initialize\n");
    result = spi_bus_add_device(spiHost_device, &z80device_config, &z80handle);
    sendMessage(result, "spi_bus_add_device\n");
}

void spi_ReadZ80Bus(void)
{
}

void app_main()
{
    gpio_pullup_en(WAITresetPIN);
    gpio_set_direction(WAITresetPIN, GPIO_MODE_OUTPUT);

    messageQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    xTaskCreate(task_SerialMessageReporter, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

    vTaskDelay(100);

    // configGPIO_IRQ();

    char myMessage[] = "tick\n";
    sendMessage(13, myMessage);
    sendMessage(12, "tock\n");
    sendAddress(10, 65000);

    config_SPIZ80();
    uint8_t RXbyte[10] ="";
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16,
        //.flags = SPI_TRANS_USE_RXDATA,
        .user = (uint8_t) RXbyte[0],
    };

    uint8_t result = 0;
    while (1)
    {
        vTaskDelay(100);

        result = spi_device_polling_transmit(z80handle, &t);

        sendMessage(result, "spi_device_poll_tx\n");
        sendAddress(0, RXbyte[0]);
        sendAddress(1, RXbyte[1]);
        sendAddress(2, RXbyte[2]);
        sendAddress(3, RXbyte[3]);
        
    }
    // gpio_set_level(IRQPIN, 1);

    // Testing GPIO speeds
    // xTaskCreate(&task_directREG, "tick_gpio", 2048, NULL, 1, NULL);
    // xTaskCreate(&task_tick_gpio, "tick_gpio", 2048, NULL, 1, NULL);
    // xTaskCreate(&task_tick_iomux, "tick_gpio", 2048, NULL, 1, NULL);
    // xTaskCreate(&task_tick_rtc, "tick_gpio", 2048, NULL, 1, NULL);

    // Running the Schedular causes more problems with timing
    // vTaskStartScheduler();
}