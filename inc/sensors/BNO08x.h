#ifndef BNO08X_H
#define BNO08X_H


#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "../../external/sh2/sh2.h"
#include "../../external/sh2/sh2_SensorValue.h"
#include "../../external/sh2/sh2_err.h"


class BNO08x
{
public:
    BNO08x(spi_inst_t* spi, uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin,
                   uint8_t cs_pin, uint32_t baudrate, uint8_t int_pin, bool spi_init );
    ~BNO08x() = default;


private:
    float x = 0;
    spi_inst_t* spi_x;

};





#endif // BNO08X_H