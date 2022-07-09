// Library headers
#include <stdio.h>
#include <string.h>


#include "driver/gpio.h"
#include "driver/spi_master.h"

void config_SPIZ80(void);
uint16_t spi_ReadZ80AddressBusPolled(void);
uint16_t spi_ReadZ80AddressBus(void);
