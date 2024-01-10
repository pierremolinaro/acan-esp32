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

static const uint32_t DESIRED_BIT_RATE = 25UL * 1000UL ;

//----------------------------------------------------------------------------------------

static volatile uint64_t gFallingEdgeTime = 0 ;
static volatile uint64_t gMinDuration = UINT64_MAX ;
static hw_timer_t * timer = nullptr ;

//----------------------------------------------------------------------------------------

void IRAM_ATTR handleEdgeInterrupt (void) {
  const uint64_t currentTime = timerRead (timer) ;
  if (gFallingEdgeTime > 0) {
    const uint64_t lowPulseDuration = currentTime - gFallingEdgeTime ;
    if (gMinDuration > lowPulseDuration) {
      gMinDuration = lowPulseDuration ;
    }
    gFallingEdgeTime = 0 ;
  }else{
    gFallingEdgeTime = currentTime ;
  }
}

//----------------------------------------------------------------------------------------
//   SETUP
//----------------------------------------------------------------------------------------

void setup() {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
//--- Configure ESP32 CAN
  Serial.println ("Configure ESP32 CAN") ;
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);
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
  } else {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  delay (100) ;

  attachInterrupt (settings.mTxPin, handleEdgeInterrupt, FALLING) ;
  timer = timerBegin (40'000'000) ;
  // 0 = first timer
  // 2 is prescaler so 80 MHz divided by 2 = 40 MHz signal
  // true - counts up
  timerStart (timer) ;
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
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Sent: ") ;
    Serial.print (gSentFrameCount) ;
    Serial.print (", receive: ") ;
    Serial.print (gReceivedFrameCount) ;
    Serial.print (", duration: ") ;
    Serial.print (gMinDuration) ;
    Serial.print (" ticks, bit rate: ") ;
    if (gMinDuration == UINT64_MAX) {
      Serial.println ("*") ;
    }else{
      Serial.print ((2 * 40 * 1000 * 1000) / gMinDuration) ;
      Serial.println (" bit/s") ;
    }
    gMinDuration = UINT64_MAX ;
    gFallingEdgeTime = 0 ;
    delay (500) ;
    frame.id = 0x555 ;
    const bool ok = ACAN_ESP32::can.tryToSend (frame) ;
    if (ok) {
      gSentFrameCount += 1 ;
    }
  }
  while (ACAN_ESP32::can.receive (frame)) {
    gReceivedFrameCount += 1 ;
  }
}
