#include "RFIDReader.h"

RFIDReader::RFIDReader(byte SS_PIN, byte RST_PIN) {
    reader = new MFRC522(SS_PIN, RST_PIN);
}

// RFIDReader::~RFIDReader(byte SS_PIN, byte RST_PIN) {
//     delete reader;
// }

void RFIDReader::readerStart() {
	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	this->PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	this->PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

void MFRC522::checkReaderData() {
  	if(this->PICC_IsNewCardPresent() && this->PICC_ReadCardSerial())
    	this->PICC_DumpToSerial(&(this->uid));
}