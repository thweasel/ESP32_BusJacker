
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
#include "interrupt.h"
#include "z80SPI.h"








void app_main()
{
    gpio_pullup_en(WAITresetPIN);
    gpio_set_direction(WAITresetPIN, GPIO_MODE_OUTPUT);

    gpio_pullup_en(5);
    gpio_set_direction(5,GPIO_MODE_OUTPUT);

    startMessageQueue();

    vTaskDelay(100);

    // configGPIO_IRQ();

    char myMessage[] = "tick\n";
    sendMessage(13, myMessage);
    sendMessage(12, "tock\n");
    sendAddress(10, 65000);

    config_SPIZ80();

    configGPIO_IRQ();

    while(1)
    {
        sendMessage(1,"tick\n");
        vTaskDelay(250);
        //WAITreset();
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