#include "RFIDReader.h"
#include "SPI.h"

RFIDReader::RFIDReader(byte SS_PIN, byte RST_PIN) {
    //MFRC522 reader(SS_PIN, RST_PIN);
    //reader = new MFRC522(SS_PIN, RST_PIN);
    reader = new MFRC522(SS_PIN, RST_PIN);
    //this->reader = mfrc522;
    this->readerStart();
}

// RFIDReader::~RFIDReader(byte SS_PIN, byte RST_PIN) {
//     delete reader;
// }

void RFIDReader::readerStart() {

    //MFRC522 mfrc522(SS_PIN, RST_PIN);

    //reader = mfrc522;
    //Serial.begin(9600);
	while(!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	reader->PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	reader->PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	//Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void RFIDReader::checkReaderData() {
  	// if(reader.PICC_IsNewCardPresent()) {
    //     Serial.println("111111111111111111111");
    // }
    // if(reader.PICC_ReadCardSerial()) {
    //     Serial.println("22222222222222222222");
    // }

    // reader.PICC_DumpDetailsToSerial(&(reader.uid));
    
    //Serial.println("CHEEEEECK");
    if(reader->PICC_IsNewCardPresent() && reader->PICC_ReadCardSerial())
        reader->PICC_DumpDetailsToSerial(&(reader->uid));
}