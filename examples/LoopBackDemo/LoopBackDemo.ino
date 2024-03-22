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
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL ; // 1 Mb/s

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup () {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
//--- Display ESP32 Chip Info
  esp_chip_info_t chip_info ;
  esp_chip_info (&chip_info) ;
  Serial.print ("ESP32 Arduino Release: ") ;
  Serial.println (ARDUINO_ESP32_RELEASE) ;
  Serial.print ("ESP32 Chip Revision: ") ;
  Serial.println (chip_info.revision) ;
  Serial.print ("ESP32 SDK: ") ;
  Serial.println (ESP.getSdkVersion ()) ;
  Serial.print ("ESP32 Flash: ") ;
  uint32_t size_flash_chip ;
  esp_flash_get_size (NULL, &size_flash_chip) ;
  Serial.print (size_flash_chip / (1024 * 1024)) ;
  Serial.print (" MB ") ;
  Serial.println (((chip_info.features & CHIP_FEATURE_EMB_FLASH) != 0) ? "(embeded)" : "(external)") ;
  Serial.print ("APB CLOCK: ") ;
  Serial.print (APB_CLK_FREQ) ;
  Serial.println (" Hz") ;
//--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE) ;
  settings.mRequestedCANMode = ACAN_ESP32_Settings::LoopBackMode ;
//  settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
//  settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
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
    Serial.print ("Distance            ") ;
    Serial.print (settings.ppmFromDesiredBitRate ()) ;
    Serial.println (" ppm") ;
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

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop () {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Sent: ") ;
    Serial.print (gSentFrameCount) ;
    Serial.print (" ") ;
    Serial.print ("Receive: ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print (" ") ;
    Serial.print (" STATUS 0x") ;
    Serial.print (TWAI_STATUS_REG, HEX) ;
    Serial.print (" RXERR ") ;
    Serial.print (TWAI_RX_ERR_CNT_REG) ;
    Serial.print (" TXERR ") ;
    Serial.println (TWAI_TX_ERR_CNT_REG) ;
    const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCount += 1 ;
  }
}

//----------------------------------------------------------------------------------------
