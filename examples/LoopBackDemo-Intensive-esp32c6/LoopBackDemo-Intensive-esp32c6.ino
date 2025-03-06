//----------------------------------------------------------------------------------------
//  Board Check: ESP32C6
//----------------------------------------------------------------------------------------

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#elif !defined (CONFIG_IDF_TARGET_ESP32C6)
  #error "Select an ESP3232C6 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32.h>

//----------------------------------------------------------------------------------------
//  Select TWAI module
//----------------------------------------------------------------------------------------

// The ESP32C6 has two CAN modules, TWAI0 and TWAI1:
//  for using TWAI0: CAN_ESP32 & myCAN = ACAN_ESP32::can
//  for using TWAI1: CAN_ESP32 & myCAN = ACAN_ESP32::can1
// See PDF documentation, section 7

ACAN_ESP32 & myCAN = ACAN_ESP32::can1 ; // Select TWAI1

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL ;

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup() {
 //--- Switch on builtin led
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
  while (!Serial) {
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE) ;
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;  // Select loopback mode
  settings.mRxPin = GPIO_NUM_17 ; // Optional, default Rx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_19 ; // Optional, default Tx pin is GPIO_NUM_5
  const uint32_t errorCode = myCAN.begin (settings) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Time Segment 1:     ") ;
    Serial.println (settings.mTimeSegment1) ;
    Serial.print ("Time Segment 2:     ") ;
    Serial.println (settings.mTimeSegment2) ;
    Serial.print ("RJW:                ") ;
    Serial.println (settings.mRJW) ;
    Serial.print ("Triple Sampling:    ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate:    ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ?    ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point:       ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
    Serial.println ("Configuration OK!");
  }else {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//----------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

static const uint32_t MESSAGE_COUNT = 10 * 1000 * 1000 ;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop() {
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("At ") ;
    Serial.print (gBlinkLedDate / 1000) ;
    Serial.print (" s, sent: ") ;
    Serial.print (gSentFrameCount) ;
    Serial.print (" ") ;
    Serial.print ("Received: ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print (" ") ;
    Serial.print (" STATUS 0x") ;
  //--- Note: TWAI register access from 3.0.0 should name the can channel
  //   < 3.0.0 :  TWAI_STATUS_REG
  //  >= 3.0.0 :  myCAN.TWAI_STATUS_REG ()
    Serial.print (myCAN.TWAI_STATUS_REG (), HEX) ;
    Serial.print (" RXERR ") ;
    Serial.print (myCAN.TWAI_RX_ERR_CNT_REG ()) ;
    Serial.print (" TXERR ") ;
    Serial.println (myCAN.TWAI_TX_ERR_CNT_REG ()) ;
  }

  CANMessage frame ;
  while (myCAN.receive (frame)) {
    gReceivedFrameCount += 1 ;
  }

  if (gSentFrameCount < MESSAGE_COUNT) {
    frame.len = 8 ;
    frame.data32 [0] = gSentFrameCount ;
    frame.data32 [1] = gReceivedFrameCount ;

    const bool ok = myCAN.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
}

//----------------------------------------------------------------------------------------
