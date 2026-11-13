#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sensors/BNO08x.h"
#include "sensors/BMP5xx.h"
#include "debug/debug.hpp"

class FlightController {
public:
    FlightController(BNO08x &bno, BMP5xx &bmp);
    ~FlightController();
    void startBNO085();
    void startBMP581();
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
    BNO08x &bno;
    BMP5xx &bmp;


};
#endif // FLIGHTCONTROLLER_H