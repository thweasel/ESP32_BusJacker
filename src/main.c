
// Library headers
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

// Project headers
#include "messageQueue.h"


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
//  SPI S-Reg Interface 
//
spi_host_device_t spiHost_device = VSPI_HOST; //HSPI_HOST;
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
    .max_transfer_sz = 64,      // Max transfer in bytes. Defaults to 4094 if 0.
    //.flags = 0,            // Abilities of bus to be checked by the driver
    //.intr_flags = 0,       // Interrupt flag for the bus to set the priority, and IRAM attribute

};

//
// SPI Device Config
//
const spi_device_interface_config_t z80device_config = {
    .command_bits = 0, // Command phase bits
    .address_bits = 0, // Address phase bits
    .dummy_bits = 0,   // Padding bits for delay before data bits
    .mode = 3, // Mode 3 Shift-Regs        // SPI mode (CPOL,CPHA?)
   
    //.clock_speed_hz = SPI_MASTER_FREQ_8M, // DEBUG - logic analyser 24Mhz max
    //.duty_cycle_pos = 64,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    //.input_delay_ns = 10, // Max valid data time, between SLCK and MISO
    
    /*
    .clock_speed_hz = SPI_MASTER_FREQ_8M, // Target speed
    .duty_cycle_pos = 32,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    */
     
    .clock_speed_hz = SPI_MASTER_FREQ_20M, // Target speed
    .duty_cycle_pos = 128,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    
    /*
    .clock_speed_hz = SPI_MASTER_FREQ_80M, // Target speed
    .duty_cycle_pos = 64,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    */
    
    .spics_io_num = -1, // GPIO pin for ChipSelect (-1 not used)
    .flags = SPI_DEVICE_NO_DUMMY,          // SPI_DEVICE_ * flags?  // SPI_DEVICE_HALFDUPLEX
    .queue_size = 1, // Transactions that can be in flight
    
    // Interupt stuff
    //.cs_ena_pretrans = 0,  // SPI bit-cycle the CS is active BEFORE Transmission
    //.cs_ena_posttrans = 0, // SPI bit-cycle the CS is active AFTER Transmission
    //.pre_cb = NULL,  // IRAM callback BEFORE Transmission
    //.post_cb = NULL, // IRAM callback AFTER Transmission
};

static uint8_t spiBufferTX[4];
static uint8_t spiBufferRX[4];

void config_SPIZ80(void)
{
    // Configure the SPI port Pins
    gpio_pullup_en(GPIO_NUM_23);
    gpio_pullup_en(GPIO_NUM_18);
    gpio_pullup_en(GPIO_NUM_19);

    //
    // INIT the SPI BUS
    //
    uint8_t result;
    result = spi_bus_initialize(spiHost_device, &mySPIBus_config, SPI_DMA_CH_AUTO);
    sendMessage(result, "spi_bus_initialize\n");

    // Add a device to the SPI BUS
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

    startMessageQueue();

    vTaskDelay(100);

    // configGPIO_IRQ();

    char myMessage[] = "tick\n";
    sendMessage(13, myMessage);
    sendMessage(12, "tock\n");
    sendAddress(10, 65000);

    config_SPIZ80();
    uint8_t TXbyte[4] = "";
    uint8_t RXbyte[4] = "";
    spi_transaction_t t1 = {
        .cmd = 0xF0,
        //.addr = 0xE0D0,
        .tx_buffer = &TXbyte,
        .rx_buffer = &RXbyte,
        .length = 16,
        .rxlength = 16,
        //.flags = SPI_TRANS_USE_RXDATA,
        //.user = (uint8_t) RXbyte[0],
    };

    uint8_t result = 0;
    result = spi_device_polling_transmit(z80handle, &t1);
    sendMessage(result, "spi_device_poll_tx\n");
    sendAddress(0, RXbyte[0]);
    sendAddress(1, RXbyte[1]);
    sendAddress(2, RXbyte[2]);
    sendAddress(3, RXbyte[3]);

    TXbyte[0] = 0x01;
    TXbyte[1] = 0x02;
    TXbyte[2] = 0x03;
    TXbyte[3] = 0x04;
    while (1)
    {
        vTaskDelay(25);

        result = spi_device_polling_transmit(z80handle, &t1);

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