#include "gpioTests.h"

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
        REG_CLR_BIT(GPIO_OUT1_REG, 0xFF);
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