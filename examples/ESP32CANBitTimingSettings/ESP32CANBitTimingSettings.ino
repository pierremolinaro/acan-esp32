/******************************************************************************/
/* File name        : ESP32ACANBitTimingSettings.ino                          */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN Bit Rate settings                             */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/


//------------------------------- Board Check ----------------------------------

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board" 
#endif

//------------------------------- Include files --------------------------------

#include <ACAN_ESP32.h>

//——————————————————————————————————————————————————————————————————————————————
//  ESP32 Desired Bit Rate
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t DESIRED_BIT_RATE = 125UL * 1000UL ; // 125 kb/s

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup() {
 //--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
 //--- Start serial
  Serial.begin (115200) ;
 //--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

 //--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);           // CAN bit rate 
  const uint16_t errorCode = ACAN_ESP32::can.begin (settings) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler : ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Time Segment 1     : ") ;
    Serial.println (settings.mTimeSegment1) ;
    Serial.print ("Time Segment 2     : ") ;
    Serial.println (settings.mTimeSegment2) ;
    Serial.print ("SJW                : ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling    : ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate    : ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ?     ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point       : ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0;

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

void loop() {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
}
