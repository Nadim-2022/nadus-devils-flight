#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "sensors/BNO08x.h"
#include "sensors/BMP5xx.h"

class FlightController {
public:
    FlightController(BNO08x &bno, BMP5xx &bmp);
    ~FlightController();
    void init();
    void startTasks();
    void stopTasks();
    void readSensors();
    void processSensorData();
    void sendTelemetry();
    void handleCommands();
    void calibrateSensors();
    void logData();
    void emergencyShutdown();
private:
    BNO08x bno;
    BMP5xx bmp;


};
#endif // FLIGHTCONTROLLER_H