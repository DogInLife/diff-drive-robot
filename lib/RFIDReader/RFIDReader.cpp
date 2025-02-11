#include "RFIDReader.h"
#include "MFRC522.h"
#include "SPI.h"

RFIDReader::RFIDReader(byte SS_PIN, byte RST_PIN) {
    //MFRC522 reader(SS_PIN, RST_PIN);
    //Serial.begin(9600);
    reader = new MFRC522(SS_PIN, RST_PIN);
    //MFRC522 reader(SS_PIN, RST_PIN);
    //Serial.begin(9600);
    //Serial.println(F("Init"));
    //this->readerStart();
}

// RFIDReader::~RFIDReader(byte SS_PIN, byte RST_PIN) {
//     delete reader;
// }

void RFIDReader::readerStart() {
    //Serial.begin(9600);
    Serial.println(F("Start"));
	if(!Serial) 
        Serial.begin(38400);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
	SPI.begin();			// Init SPI bus
	reader->PCD_Init();		// Init MFRC522
	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
	//this->reader->PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
	//Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
}

int RFIDReader::checkReaderData() {
    //long t_start = millis();
    //int dt = 0;
  	// if(reader.PICC_IsNewCardPresent()) {
    //     Serial.println("111111111111111111111");
    // }
    // if(reader.PICC_ReadCardSerial()) {
    //     Serial.println("22222222222222222222");
    // }

    // reader.PICC_DumpDetailsToSerial(&(reader.uid));
    
    //Serial.println("CHEEEEECK");
    
    // while(millis() - t_start < del) {
    // //while(millis()-t_start < del) {
    //     //Serial.println("CHECK");
    //     if(reader->PICC_IsNewCardPresent() && reader->PICC_ReadCardSerial()) {
    //         //this->reader->PICC_DumpToSerial(&(this->reader->uid));
    //         getUID();
    //         //break;
    //     }

    //     Serial.println(String(millis()));
    // }

    //int rfidFound = 0;

    if(reader->PICC_IsNewCardPresent() && reader->PICC_ReadCardSerial()) {
        //getUID();
        //Serial.println(rfidFound);
        return getUID();
    } else {
        return 0;
    }
    
}

int RFIDReader::getUID() {
    //this->reader->PICC_DumpDetailsToSerial(&(this->reader->uid));
    MFRC522::Uid *thisUid = &(reader->uid);
    
    String uidStr = "";
    for(byte i = 0; i < thisUid->size; i++) {
		// if(thisUid->uidByte[i] < 0x10)
		// 	// Serial.print(F(" 0")); // ####
		// 	//Serial.print(F("0")); // ###
        //     //uidStr = uidStr + "0";
		// else
		// 	//Serial.print(F(" ")); // ###
        uidStr = uidStr + String(thisUid->uidByte[i], HEX);
		//Serial.print(thisUid->uidByte[i], HEX);
    }

    Serial.println(uidStr);

    int rfidFound;

    if(uidStr.equals("87eafa67")) {
        rfidFound = 1;
    } else if(uidStr.equals("d4a1a733")) {
        rfidFound = 2;
    } else if(uidStr.equals("49409e5")) {
        rfidFound = 3;
    } else if(uidStr.equals("57aba285")) {
        rfidFound = 4;
    } else rfidFound = -1;

    // switch(uidStr) {
    //     case("87eafa67"):
    //         rfidFound = 1;
    //         break;
    //     case("bc20eb30"):
    //         rfidFound = 2;
    //         break;
    // }

    reader->PICC_HaltA();
    return rfidFound;
}