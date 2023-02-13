#ifndef RFID_READER_H
#define RFID_READER_H
#include <hallSensor.h>

class HallSensorReader {
    private:
        byte DIG_PIN;

    public:
        HallSensorReader(byte DIG_PIN);

        void readerStart();
        int checkReaderData();
};

#endif