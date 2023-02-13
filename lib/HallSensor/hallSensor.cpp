#include "hallSensor.h"
#include "SPI.h"

HallSensorReader::HallSensorReader(byte DIG_PIN) {
    this->DIG_PIN = DIG_PIN;
}

void HallSensorReader::readerStart() {
    pinMode(DIG_PIN, INPUT);
	if(!Serial) 
        Serial.begin(9600);		// Do nothing if no serial port is opened
}

int HallSensorReader::checkReaderData() {
    return digitalRead(DIG_PIN);
}