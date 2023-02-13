#ifndef HALL_SENSOR_READER_H
#define HALL_SENSOR_READER_H
#include <Arduino.h>
#include "SPI.h"

class HallSensorReader {
    private:
        int senPin;
        int ledPin;
        int averageSignal;
        int sensitivity;

    public:
        HallSensorReader(int SEN_PIN, int LED_PIN, int averageSignal, int sensitivity, float shift_X, float shift_Y);

        byte checkMagneticField();
        void updateAverageSignal();
        void takeOffLed();
        float x, y;
};

#endif