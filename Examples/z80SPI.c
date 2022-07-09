//
//  SPI S-Reg Interface
//
spi_host_device_t spiHost_device = VSPI_HOST; // HSPI_HOST;
spi_device_handle_t z80device_handle;

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
    .mode = 3,         // Mode 3 Shift-Regs        // SPI mode (CPOL,CPHA?)

    //.clock_speed_hz = SPI_MASTER_FREQ_8M, // DEBUG - logic analyser 24Mhz max
    //.duty_cycle_pos = 64,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    //.input_delay_ns = 10, // Max valid data time, between SLCK and MISO

    /*
    .clock_speed_hz = SPI_MASTER_FREQ_8M, // Target speed
    .duty_cycle_pos = 32,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    */

    .clock_speed_hz = SPI_MASTER_FREQ_20M, // Target speed
    .duty_cycle_pos = 128,                 // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0,                   // Max valid data time, between SLCK and MISO

    /*
    .clock_speed_hz = SPI_MASTER_FREQ_80M, // Target speed
    .duty_cycle_pos = 64,   // Duty cycle of positive clock in 1/256th (128 = 50/50) [0==128]
    .input_delay_ns = 0, // Max valid data time, between SLCK and MISO
    */

    .spics_io_num = -1,           // GPIO pin for ChipSelect (-1 not used)
    .flags = SPI_DEVICE_NO_DUMMY, // SPI_DEVICE_ * flags?  // SPI_DEVICE_HALFDUPLEX
    .queue_size = 1,              // Transactions that can be in flight

    // Interupt stuff
    //.cs_ena_pretrans = 0,  // SPI bit-cycle the CS is active BEFORE Transmission
    //.cs_ena_posttrans = 0, // SPI bit-cycle the CS is active AFTER Transmission
    //.pre_cb = NULL,  // IRAM callback BEFORE Transmission
    //.post_cb = NULL, // IRAM callback AFTER Transmission

    
    // FLAG OPTIONS
    /*
        SPI_DEVICE_TXBIT_LSBFIRST ///< Transmit command/address/data LSB first instead of the default MSB first
        SPI_DEVICE_RXBIT_LSBFIRST ///< Receive data LSB first instead of the default MSB first
        SPI_DEVICE_BIT_LSBFIRST   ///< Transmit and receive LSB first
        SPI_DEVICE_3WIRE          ///< Use MOSI (=spid) for both sending and receiving data
        SPI_DEVICE_POSITIVE_CS    ///< Make CS positive during a transaction instead of negative
        SPI_DEVICE_HALFDUPLEX     ///< Transmit data before receiving it, instead of simultaneously
        SPI_DEVICE_NO_DUMMY
        SPI_DEVICE_DDRCLK
    */
    // SPI_DEVICE_CLK_AS_CS       ///< Output clock on CS line if CS is active
    /** There are timing issue when reading at high frequency (the frequency is related to whether iomux pins are used, valid time after slave sees the clock).
     *     - In half-duplex mode, the driver automatically inserts dummy bits before reading phase to fix the timing issue. Set this flag to disable this feature.
     *     - In full-duplex mode, however, the hardware cannot use dummy bits, so there is no way to prevent data being read from getting corrupted.
     *       Set this flag to confirm that you're going to work with output only, or read without dummy bits at your own risk.
     */

};

//
//  SPI Transaction
//
uint8_t TXbyte[4] = "";
uint8_t RXbyte[4] = "";
static spi_transaction_t t1 = {
    .cmd = 0xF0,
    //.addr = 0xE0D0,
    .tx_buffer = &TXbyte,
    .rx_buffer = &RXbyte,
    .length = 16,
    .rxlength = 16,
    //.flags = SPI_TRANS_USE_RXDATA,
    //.user = (uint8_t) RXbyte[0],
};

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
    result = spi_bus_add_device(spiHost_device, &z80device_config, &z80device_handle);
    sendMessage(result, "spi_bus_add_device\n");
}

void spi_ReadZ80Bus(void)
{
}