#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"
//#include "MFRC522.h"
#include "RFIDReader.h"

#define RST_PIN         5          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above


//Serial.begin(9600);
RFIDReader rfidReader(SS_PIN, RST_PIN);

// #include <SPI.h>
//#include <MFRC522.h>

// MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

byte del = 50; // задержка
int whl_vel_des = 60; // скорость колеса [об/мин]
bool deb = false; // флаг типа дебаггинга

//float R = 0.5;

float KpL = 0.0;
float KiL = 0.0;
float KdL = 0.0;
float KpR = 0.0;
float KiR = 0.0;
float KdR = 0.0;

void setup() {

  //Serial.begin(9600);
  Serial.begin(9600);
  Serial.println("SETUP");
  //reader->readerStart();
  
//   TwoWheeledRobot robot;
//   robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
//   robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
//   robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
//   //robot.tunePID(5.3, 4.8, 0);
//   //robot.tunePID(0.7, 1.5, 0.0);
//   //robot.tunePID(4, 1.5, 0); // РАБОТАЕТ НА КРУГЕ (ну так)
//   robot.tunePID(3.5, 1.8, 0);
// // ====== Д Л Я  120 ОБ/МИН ======
// //   KpL = 600.0;
// //   KiL = 12000.0;
// //   KdL = 0.5;
// // // ================  П Р О В Е Р Ь  ============
// // // ================     Ф Л А Г     ============
// // // ================      D E B      ============
// //   KpR = 600.0;
// //   KiR = 12000.0;
// //   KdR = 0.5;

// // ====== Д Л Я 60  ОБ/МИН =====
//   KpL = 250.0;
//   KiL = 5000.0;
//   KdL = 0.5;
// // ================  П Р О В Е Р Ь  ============
// // ================     Ф Л А Г     ============
// // ================      D E B      ============
//   KpR = 250.0;
//   KiR = 5000.0;
//   KdR = 0.5;
//   robot.tuneWhlPID(KpL, KiL, KdL, KpR, KiR, KdR);
//   robot.serialControl(deb);
//   //robot.goCircle(1.0, 8);
//   //robot.manualControl(del);
//   //robot.rot_test(whl_vel_des, del, deb); // ########################

//   // float xGoal = 1;
//   // float yGoal = 1;
//   // robot.goToGoal(xGoal, yGoal, dt);
//   // robot.manualControl();
}


void loop() {
  rfidReader.checkReaderData();
}


// #include <SPI.h>
// #include <MFRC522.h>

// #define RST_PIN         5          // Configurable, see typical pin layout above
// #define SS_PIN          53         // Configurable, see typical pin layout above

// MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

// void setup() {
// 	Serial.begin(9600);		// Initialize serial communications with the PC
// 	while (!Serial);		// Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
// 	SPI.begin();			// Init SPI bus
// 	mfrc522.PCD_Init();		// Init MFRC522
// 	delay(4);				// Optional delay. Some board do need more time after init to be ready, see Readme
// 	mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
// 	Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
// }

// void loop() {

// 	// // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
// 	// if ( ! mfrc522.PICC_IsNewCardPresent()) {
// 	// 	return;
// 	// }

// 	// // Select one of the cards
// 	// if ( ! mfrc522.PICC_ReadCardSerial()) {
// 	// 	return;
// 	// }

// 	// // Dump debug info about the card; PICC_HaltA() is automatically called
// 	// mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

//   if(mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
//     mfrc522.PICC_DumpToSerial(&(mfrc522.uid));
//   }
// }
