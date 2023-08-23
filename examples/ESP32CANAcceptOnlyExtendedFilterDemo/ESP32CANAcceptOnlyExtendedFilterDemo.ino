//----------------------------------------------------------------------------------------
// This sketch is a demo for "accept only extented frames" receive filter
// It runs in any mode, this sketch uses LoopBack mode
// In LoopBack mode, the ESP32 requires to be connected to a CAN transceiver (see documentation)
//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32.h>

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL ;

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup () {
//--- Switch on builtin led
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
//--- Configure ESP32 CAN
  Serial.println("Configure ESP32 CAN");
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE) ;
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
//  settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
//  settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
  const ACAN_ESP32_Filter filter = ACAN_ESP32_Filter::acceptExtendedFrames () ;
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings, filter);
  if (errorCode == 0) {
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);
    Serial.print ("RJW:                ") ;
    Serial.println (settings.mRJW) ;
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
    Serial.println("Configuration OK!");
  }else{
    Serial.print ("Configuration error 0x");
    Serial.println (errorCode, HEX);
  }
}

//----------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0 ;
static uint16_t gPhase1 = 0 ;
static uint32_t gPhase2 = 0 ;

static const uint16_t HIGHEST_STANDARD_IDENTIFIER = (1 << 11) - 1 ;
static const uint32_t HIGHEST_EXTENDED_IDENTIFIER = (1 << 29) - 1 ;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop() {
  if (gBlinkLedDate < millis()) {
    gBlinkLedDate += 1000 ;
    digitalWrite(LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    if ((gBlinkLedDate % (60 * 1000)) == 0) {
      Serial.print ("At ") ;
      Serial.print (gBlinkLedDate / (60 * 1000)) ;
      Serial.print (" min, sending state: phase1 0x") ;
      Serial.print (gPhase1, HEX) ;
      Serial.print (", phase2 0x") ;
      Serial.println (gPhase2, HEX) ;
    }
  }
//--- Send standard frame
  CANMessage frame;
  if (gPhase1 <= (HIGHEST_STANDARD_IDENTIFIER * 2 + 1)) {
    frame.id = gPhase1 >> 1 ;
    frame.rtr = (gPhase1 & 1) != 0 ;
    if (ACAN_ESP32::can.tryToSend (frame)) {
      gPhase1 += 1 ;
    }
  }else if (gPhase2 <= (HIGHEST_EXTENDED_IDENTIFIER * 2 + 1)) {
    frame.ext = true ;
    frame.id = gPhase2 >> 1;
    frame.rtr = (gPhase2 & 1) != 0 ;
    if (ACAN_ESP32::can.tryToSend (frame)) {
      gPhase2 += 1 ;
    }
  }
 //--- Received frame ?
  if (ACAN_ESP32::can.receive (frame)) {
    Serial.print ("**** Received ") ;
    Serial.print (frame.ext ? "extended " : "standard ") ;
    Serial.print (frame.rtr ? "remote " : "data ") ;
    Serial.print ("frame, id 0x") ;
    Serial.println (frame.id, HEX) ;
  }
}

//----------------------------------------------------------------------------------------
