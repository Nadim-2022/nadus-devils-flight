#include "flightcontroller/Flightcontroller.h"

FlightController::FlightController(BNO08x &bno, BMP5xx &bmp) : bno(bno), bmp(bmp) 
{
}

void startBNO085() {
    // Start the BNO085 sensor
    if (!bno.initialize()) {
        DEBUG_PRINTF("Failed to find BNO08x\n");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    DEBUG_PRINTF("BNO08x found\n");
    for (int n = 0; n < bno.prodIds.numEntries; n++) {
        DEBUG_PRINTF("Part %lu: v%u.%u.%u build %lu\n",
            (unsigned long)bno.prodIds.entry[n].swPartNumber,
            bno.prodIds.entry[n].swVersionMajor,
            bno.prodIds.entry[n].swVersionMinor,
            bno.prodIds.entry[n].swVersionPatch,
            (unsigned long)bno.prodIds.entry[n].swBuildNumber);
    }

    bno.setDcdAutoSave(false); 
    if (!bno.enableReport(SH2_ROTATION_VECTOR, 5000)) { 
        DEBUG_PRINTF("Could not enable rotation vector\n");
    }
}

void startBMP581() 
{

    if (!bmp.initialize()) 
    {
        DEBUG_PRINTF("BMP5xx not found, status %d\n", bmp.lastStatus());
    }
    
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);   
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_127);   
    bmp.setOutputDataRate(BMP5XX_ODR_240_HZ);  
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

    
    bmp.enablePressure(true);

   
    bmp.configureInterrupt(BMP5XX_INTERRUPT_PULSED,
                            BMP5XX_INTERRUPT_ACTIVE_HIGH,
                            BMP5XX_INTERRUPT_PUSH_PULL,
                            BMP5XX_INTERRUPT_DATA_READY,
                            true);
    DEBUG_PRINTF("============================================================\n");
    DEBUG_PRINTF("Current settings:\n");
    DEBUG_PRINTF("  Temperature OSR: %d\n", bmp.getTemperatureOversampling());
    DEBUG_PRINTF("  Pressure OSR: %d\n", bmp.getPressureOversampling());
    DEBUG_PRINTF("  IIR Filter Coeff: %d\n", bmp.getIIRFilterCoeff());
    DEBUG_PRINTF("  Output Data Rate: %d\n", bmp.getOutputDataRate());
    DEBUG_PRINTF("============================================================\n");
    
}