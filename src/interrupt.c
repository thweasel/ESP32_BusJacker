#include "interrupt.h"
#include "z80SPI.h"
#include "messageQueue.h"

uint16_t address = 0;



//
//  Latch Bus to Shift Register
//

#define LOADregPINbit (32) // GPIO 5
void LOADreg(void)
{
    REG_CLR_BIT(GPIO_OUT_REG, LOADregPINbit);
    REG_SET_BIT(GPIO_OUT_REG, LOADregPINbit);
}

//
//  Z80 WAIT signal
//

// #define WAITresetPIN 33
void WAITreset(void)
{
    REG_CLR_BIT(GPIO_OUT1_REG, 2);

    int16_t i = 0;
    while (i < 48) // 48
        i++;

    REG_SET_BIT(GPIO_OUT1_REG, 2);
}

//
//  Interrupt Service
//

void task_IRQ(void)
{
    
    sendMessage(2,"ISR\n");

    // Latch Address BUS
    LOADreg();

    // SPI load Address
    address = spi_ReadZ80AddressBus();

    // Address
    sendAddressISR(0, address);

    // release WAIT
    WAITreset();
    vTaskDelete(NULL);
}

void task_AddressBusRead(void)
{
    address = spi_ReadZ80AddressBus();
    sendAddressISR(0,address);
    vTaskDelete(NULL);
}

#define IRQPIN 32
void IRAM_ATTR myIRQ(void *arg)
{    
    LOADreg();
    
    address = spi_ReadZ80AddressBusPolled();
    
    sendAddressISR(0,address);
    
    WAITreset();
    
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
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
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