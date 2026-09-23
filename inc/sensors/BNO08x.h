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
    BNO08x();
    

private:
    float x = 0;

};





#endif // BNO08X_H