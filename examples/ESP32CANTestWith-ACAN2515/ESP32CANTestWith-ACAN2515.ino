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
#include <ACAN2515.h>

//----------------------------------------------------------------------------------------

static const byte MCP2515_SCK = 26 ; // SCK input of MCP2515
static const byte MCP2515_SDI = 19 ; // SI input of MCP2515
static const byte MCP2515_SDO = 18 ; // SO output of MCP2515

static const byte MCP2515_CS  = 17 ; // CS input of MCP2515
static const byte MCP2515_INT = 23 ; // INT output of MCP2515
static const byte MCP2515_RESET = 27 ; // RESET input of MCP2515 (adapt to your design)

//----------------------------------------------------------------------------------------
//  MCP2515 Driver object
//----------------------------------------------------------------------------------------

ACAN2515 can2515 (MCP2515_CS, SPI, MCP2515_INT) ;

//----------------------------------------------------------------------------------------
//  MCP2515 Quartz: adapt to your design
//----------------------------------------------------------------------------------------

static const uint32_t MCP2515_QUARTZ_FREQUENCY = 20UL * 1000UL * 1000UL ; // 20 MHz

//----------------------------------------------------------------------------------------
// Desired Bit Rate
//----------------------------------------------------------------------------------------

static const uint32_t DESIRED_BIT_RATE = 1000UL * 1000UL ; // 1 Mb/s

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup() {
//--- RESET MCP2515
  pinMode (MCP2515_RESET, OUTPUT) ;
  digitalWrite (MCP2515_RESET, LOW) ;
  delay (10) ;
  digitalWrite (MCP2515_RESET, HIGH) ;
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
//--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);           // CAN bit rate
//  settings.mRxPin = GPIO_NUM_4 ; // Optional, default Tx pin is GPIO_NUM_4
//  settings.mTxPin = GPIO_NUM_5 ; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings) ;
  if (errorCode == 0) {
    Serial.println ("Configuration ESP32 OK!");
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  //--- Begin SPI1
  SPI.begin (MCP2515_SCK, MCP2515_SDO, MCP2515_SDI) ;
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings2515 (MCP2515_QUARTZ_FREQUENCY, DESIRED_BIT_RATE) ;
  const uint32_t errorCode2515 = can2515.begin (settings2515, [] { can2515.isr () ; }) ;
  if (errorCode2515 == 0) {
    Serial.println ("ACAN2515 configuration: ok") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode2515, HEX) ;
  }
}

//----------------------------------------------------------------------------------------

static uint32_t gBlinkLedDate = 0;
static uint32_t gReceivedFrameCount2515 = 0 ;
static uint32_t gReceivedFrameCountESP32 = 0 ;
static uint32_t gSentFrameCount2515 = 0 ;
static uint32_t gSentFrameCountESP32 = 0 ;

static const uint32_t MESSAGE_COUNT = 10 * 1000 ;

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------

void loop() {

  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print("SentESP32: ");
    Serial.print(gSentFrameCountESP32);
    Serial.print(" ");
    Serial.print("Receive2515: ");
    Serial.print(gReceivedFrameCount2515);
    Serial.print(" ");
    Serial.print("Sent2515: ");
    Serial.print(gSentFrameCount2515);
    Serial.print(" ");
    Serial.print("ReceiveESP32: ");
    Serial.println(gReceivedFrameCountESP32);
  }

  CANMessage frame ;

  if (gSentFrameCountESP32 < MESSAGE_COUNT) {
    //frame.ext=true;
    frame.id= millis () & 0x7FE;
    frame.len = 8;
    const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCountESP32 += 1 ;
    }
  }

  if (gSentFrameCount2515 < MESSAGE_COUNT) {
  //--- Make an odd identifier for 2515
    //frame.ext=true;
    frame.id = millis () & 0x7FE ;
    frame.id |= 1 ;
    frame.len = 8 ;
  //--- Send frame via the MCP2515, using transmit buffer 0
    bool ok = can2515.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount2515 += 1 ;
    }
  }

  while (can2515.receive (frame)) {
    gReceivedFrameCount2515 += 1 ;
  }
  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCountESP32 += 1 ;
  }
}

//----------------------------------------------------------------------------------------
