#include "sensors/BNO08x.h"


BNO08x::BNO08x(spi_inst_t* spi, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin,
                   uint8_t cs_pin, uint32_t baudrate, uint8_t int_pin, bool spix_init):spi_x(spi)
{
    if (spix_init){
        spi_init(spi, baudrate);
        spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
        
        gpio_set_function(sck_pin, GPIO_FUNC_SPI);
        gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    }
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
}