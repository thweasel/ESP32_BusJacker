#include "interrupt.h"

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
//  Latch Bus to Shift Register
//

#define LOADregPINbit 2 ^ 5 //GPIO 5
void LOADreg()
{
    REG_CLR_BIT(GPIO_OUT_REG, LOADregPINbit);
    REG_SET_BIT(GPIO_OUT_REG, LOADregPINbit);
}

//
//  Interrupt Service
//

#define IRQPIN 32
void IRAM_ATTR myIRQ(void *arg)
{
    uint16_t address;

    // Latch Address BUS
    LOADreg();

    // SPI load Address
    

    // Merge bytes


    // Address
    sendAddress(0,sendAddress);


    WAITreset();

    AMessage testMessage;
    testMessage.ucMessageID = IRQPIN;
    strcpy(testMessage.ucData, "IRQ\n");
    xQueueSendFromISR(messageQueue, &testMessage, NULL);

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

    switch (result)
    {
    case ESP_OK:
        sendMessage(0, "ESP_OK\n");
        break;
    case ESP_ERR_INVALID_STATE:
        sendMessage(0, "ESP_ERR_INVALID_STATE\n");
        break;
    case ESP_ERR_INVALID_ARG:
        sendMessage(0, "ESP_ERR_INVALID_ARG\n");
        break;
    default:
        break;
    }
}