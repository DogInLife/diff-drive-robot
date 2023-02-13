#include "hallSensor.h"

HallSensorReader::HallSensorReader(int SEN_PIN, int LED_PIN, int averageSignal, int sensitivity, float shift_X, float shift_Y) {
    senPin = SEN_PIN;
    ledPin = LED_PIN;
    x = shift_X;
    y = shift_Y;
    this->averageSignal = averageSignal;
    this->sensitivity = sensitivity;

    pinMode(ledPin, INPUT);
}

void HallSensorReader::updateAverageSignal () {
    averageSignal = analogRead(senPin);
    //Serial.print(String(averageSignal) + ", ");
}

byte HallSensorReader::checkMagneticField() {
    byte signal_val = abs(analogRead(senPin)-averageSignal) > sensitivity;
    //Serial.print(analogRead(senPin)-averageSignal); 
    //Serial.print(" "); 
    digitalWrite(ledPin, signal_val ? HIGH : LOW);
    return signal_val;
}

void HallSensorReader::takeOffLed() {
     digitalWrite(ledPin, LOW);
}