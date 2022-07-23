#ifndef RFID_READER_H
#define RFID_READER_H
#include <MFRC522.h>

class RFIDReader {
    private:
        MFRC522 reader;

    public:
        RFIDReader(byte SS_PIN, byte RST_PIN);
        //~RFIDReader(byte SS_PIN, byte RST_PIN);

        void readerStart();
        void checkReaderData();
};

#endif