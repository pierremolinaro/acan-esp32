/******************************************************************************/
/* File name        : ACAN_ESP32.cpp                                          */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN Driver                                        */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/
/*  Version | Change                                                          */
/* ---------------------------------------------------------------------------*/
/*   V1.0   | Creation                                                        */
/*   V1.1   | Working by Polling (Extended Frames)                            */
/*   V1.2   | Handling Standard Frames                                        */
/*   V1.3   | Added Interrupt Handlers                                        */
/*   V2.0   | Acceptance Filter Settings                                      */
/* ---------------------------------------------------------------------------*/

//------------------------------- Include files ----------------------------------------------------

#include <ACAN_ESP32.h>

//--------------------------------------------------------------------------------------------------
//   ESP32 Critical Section
//--------------------------------------------------------------------------------------------------

// taskENTER_CRITICAL() of FREE-RTOS is deprecated as portENTER_CRITICAL() in ESP32
//--- https://esp32.com/viewtopic.php?t=1703
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED ;

//--------------------------------------------------------------------------------------------------
//   CONSTRUCTOR
//--------------------------------------------------------------------------------------------------

ACAN_ESP32::ACAN_ESP32 (void) :
mAcceptedFrameFormat (ACAN_ESP32_Filter::standardAndExtended),
mDriverReceiveBuffer (),
mDriverTransmitBuffer (),
mDriverIsSending (false) {
}

//--------------------------------------------------------------------------------------------------
//   Set the GPIO pins
//--------------------------------------------------------------------------------------------------

void ACAN_ESP32::setGPIOPins (const gpio_num_t inTXPin,
                              const gpio_num_t inRXPin) {
//--- Set TX pin
  pinMode (inTXPin, OUTPUT) ;
  pinMatrixOutAttach (inTXPin, CAN_TX_IDX, false, false) ;
//--- Set RX pin
  pinMode (inRXPin, INPUT) ;
  pinMatrixInAttach (inRXPin, CAN_RX_IDX, false) ;
}

//--------------------------------------------------------------------------------------------------
//   Set the Requested Mode
//--------------------------------------------------------------------------------------------------

void ACAN_ESP32::setRequestedCANMode (const ACAN_ESP32_Settings & inSettings,
                                      const ACAN_ESP32_Filter & inFilter) {
// ESP32 CAN Operation Mode Configuration
//
//    Supported Mode                 MODE Registers
//     - Normal Mode                  - Reset             -> bit(0)
//     - No ACK                       - ListenOnly        -> bit(1)
//     - Acceptance Filter            - SelfTest          -> bit(2)
//                                    - Acceptance Filter -> bit(3)

  uint8_t requestedMode = 0 ;
  switch (inSettings.mRequestedCANMode) {
    case ACAN_ESP32_Settings::NormalMode :
      break ;
    case ACAN_ESP32_Settings::ListenOnlyMode :
      requestedMode = CAN_MODE_LISTENONLY ;
      break ;
    case ACAN_ESP32_Settings::LoopBackMode :
      requestedMode = CAN_MODE_SELFTEST ;
      break ;
  }

  if (inFilter.mAMFSingle) {
    requestedMode |= CAN_MODE_ACCFILTER ;
  }

  CAN_MODE = requestedMode | CAN_MODE_RESET ;

  do{
    CAN_MODE = requestedMode ;
  }while ((CAN_MODE & CAN_MODE_RESET) != 0) ;
}

//--------------------------------------------------------------------------------------------------
//   Set the Bus timing Registers
//--------------------------------------------------------------------------------------------------

inline void ACAN_ESP32::setBitTimingSettings (const ACAN_ESP32_Settings & inSettings) {
// BUS TIMING Configuration of ESP32 CAN
//     ACAN_ESP32_Settings calculates the best values for the desired bit Rate.
//
//  BTR0 : bit (0 - 5) -> Baud Rate Prescaller (BRP)
//         bit (6 - 7) -> Resynchronization Jump Width (RJW)
//
//  BTR1 : bit (0 - 3) -> TimeSegment 1 (Tseg1)
//         bit (4 - 6) -> TimeSegment 2 (Tseg2)
//         bit (7)     -> TripleSampling? (SAM)

  CAN_BTR0 = ((inSettings.mRJW - 1) << 6) |            // SJW
             ((inSettings.mBitRatePrescaler - 1) << 0) // BRP
  ;

  CAN_BTR1 = ((inSettings.mTripleSampling) << 7)   | // Sampling
             ((inSettings.mTimeSegment2 - 1) << 4) | // Tseg2
             ((inSettings.mTimeSegment1 - 1) << 0)   // Tseg1
  ;
}

//--------------------------------------------------------------------------------------------------

void ACAN_ESP32::setAcceptanceFilter (const ACAN_ESP32_Filter & inFilter) {
//--- Write the Code and Mask Registers with Acceptance Filter Settings
  if (inFilter.mAMFSingle) {
    CAN_MODE |= CAN_MODE_ACCFILTER ;
  }
  mAcceptedFrameFormat = inFilter.mFormat ;

  CAN_ACC_CODE_FILTER (0) = inFilter.mACR0 ;
  CAN_ACC_CODE_FILTER (1) = inFilter.mACR1 ;
  CAN_ACC_CODE_FILTER (2) = inFilter.mACR2 ;
  CAN_ACC_CODE_FILTER (3) = inFilter.mACR3 ;

  CAN_ACC_MASK_FILTER (0) = inFilter.mAMR0 ;
  CAN_ACC_MASK_FILTER (1) = inFilter.mAMR1 ;
  CAN_ACC_MASK_FILTER (2) = inFilter.mAMR2 ;
  CAN_ACC_MASK_FILTER (3) = inFilter.mAMR3 ;

}

//--------------------------------------------------------------------------------------------------
//   BEGIN
//--------------------------------------------------------------------------------------------------

uint32_t ACAN_ESP32::begin (const ACAN_ESP32_Settings & inSettings,
                            const ACAN_ESP32_Filter & inFilterSettings) {
  uint32_t errorCode = 0 ; // Ok be default
//--- Enable CAN module
  //----Access the CAN Peripheral registers and initialize the CLOCK
  //https://github.com/ThomasBarth/ESP32-CAN-Driver/blob/master/components/can/CAN.c
  //Function periph_module_enable(); - https://github.com/espressif/esp-idf/blob/master/components/driver/periph_ctrl.c
  periph_module_enable (PERIPH_CAN_MODULE);
//--- Set GPIO pins
  setGPIOPins (inSettings.mTxPin, inSettings.mRxPin);
//--------------------------------- Required: It is must to enter RESET Mode to write the Configuration Registers
  while ((CAN_MODE & CAN_MODE_RESET) == 0) {
    CAN_MODE = CAN_MODE_RESET ;
  }
  if ((CAN_MODE & CAN_MODE_RESET) == 0) {
    errorCode = kNotInRestModeInConfiguration ;
  }
//--------------------------------- Use Pelican Mode
  CAN_CLK_DIVIDER = CAN_PELICAN_MODE;
//---- Check the Register access and bit timing settings before writing to the Bit Timing Registers
  CAN_BTR0 = 0x55 ;
  bool ok = CAN_BTR0 == 0x55 ;
  if (ok) {
    CAN_BTR0 = 0xAA ;
    ok = CAN_BTR0 == 0xAA ;
  }
  if (!ok) {
    errorCode |= kCANRegistersError ;
  }
//----------------------------------- If ok, check the bit timing settings are correct
  if (!inSettings.mBitRateClosedToDesiredRate) {
    errorCode |= kTooFarFromDesiredBitRate;
  }
  errorCode |= inSettings.CANBitSettingConsistency ();
//----------------------------------- Allocate buffer
  if (!mDriverReceiveBuffer.initWithSize (inSettings.mDriverReceiveBufferSize)) {
    errorCode |= kCannotAllocateDriverReceiveBuffer ;
  }
  if (!mDriverTransmitBuffer.initWithSize (inSettings.mDriverTransmitBufferSize)) {
    errorCode |= kCannotAllocateDriverTransmitBuffer ;
  }
//--------------------------------- Set Bus timing Registers
  if (errorCode == 0) {
    setBitTimingSettings (inSettings) ;
  }
//--------------------------------- Set the Acceptance Filter
  setAcceptanceFilter (inFilterSettings) ;
//--------------------------------- Set and clear the error counters to default value
  CAN_EWLR = 96 ;
  CAN_RX_ECR = 0 ;
  CAN_TX_ECR = 0 ;
//--------------------------------- Clear the Interrupt Registers
  const uint8_t unusedVariable __attribute__((unused)) = CAN_INTERRUPT ;
//--------------------------------- Set Interrupt Service Routine
  esp_intr_alloc (ETS_CAN_INTR_SOURCE, 0, isr, this, nullptr) ;
//--------------------------------- Enable Interupts
  CAN_IER = CAN_INTERRUPT_TX_ENABLE | CAN_INTERRUPT_RX_ENABLE ;
//--------------------------------- Set to Requested Mode
  setRequestedCANMode (inSettings, inFilterSettings) ;
//---
  return errorCode ;
}

//--------------------------------------------------------------------------------------------------
//   Interrupt Handler
//--------------------------------------------------------------------------------------------------

void IRAM_ATTR ACAN_ESP32::isr (void * inUserArgument) {
  ACAN_ESP32 * myDriver = (ACAN_ESP32 *) inUserArgument ;

  portENTER_CRITICAL (&mux) ;
  const uint32_t interrupt = CAN_INTERRUPT ;
  if ((interrupt & CAN_INTERRUPT_RX) != 0) {
     myDriver->handleRXInterrupt();
  }
  if ((interrupt & CAN_INTERRUPT_TX) != 0) {
     myDriver->handleTXInterrupt();
  }
  portEXIT_CRITICAL (&mux) ;

  portYIELD_FROM_ISR () ;
}

//-------------------------------------------------------------------------------------

void ACAN_ESP32::handleTXInterrupt (void) {
  CANMessage message ;
  const bool sendmsg = mDriverTransmitBuffer.remove (message) ;
  if (sendmsg) {
    internalSendMessage (message);
  }else {
    mDriverIsSending = false ;
  }
}

//-------------------------------------------------------------------------------------

void ACAN_ESP32::handleRXInterrupt (void) {
  CANMessage frame;
  getReceivedMessage (frame) ;
  switch (mAcceptedFrameFormat) {
  case ACAN_ESP32_Filter::standard :
    if (!frame.ext) {
      mDriverReceiveBuffer.append (frame) ;
    }
    break ;
  case ACAN_ESP32_Filter::extended :
    if (frame.ext) {
      mDriverReceiveBuffer.append (frame) ;
    }
    break ;
  case ACAN_ESP32_Filter::standardAndExtended :
    mDriverReceiveBuffer.append (frame) ;
    break ;
  }
}

//--------------------------------------------------------------------------------------------------
//   RECEPTION
//--------------------------------------------------------------------------------------------------

bool ACAN_ESP32::receive (CANMessage & outMessage) {
  portENTER_CRITICAL (&mux) ;
    const bool hasReceivedMessage = mDriverReceiveBuffer.remove (outMessage) ;
  portEXIT_CRITICAL (&mux) ;
  return hasReceivedMessage ;
}

//--------------------------------------------------------------------------------------------------

void ACAN_ESP32::getReceivedMessage (CANMessage & outFrame) {
  const uint32_t frameInfo = CAN_FRAME_INFO ;

  outFrame.len = frameInfo & 0xF;
  if (outFrame.len > 8) {
    outFrame.len = 8 ;
  }
  outFrame.rtr = (frameInfo & CAN_RTR) != 0;
  outFrame.ext = (frameInfo & CAN_FRAME_FORMAT_EFF) != 0 ;

  //-----------Standard Frame
  if(!outFrame.ext) {
    uint32_t identifier =  uint32_t (CAN_ID_SFF(0)) << 3 ;
             identifier |= uint32_t (CAN_ID_SFF(1)) >> 5 ;
    outFrame.id = identifier;

    for (uint8_t i=0 ; i<outFrame.len ; i++) {
      outFrame.data[i] = CAN_DATA_SFF(i);
    }
  }else { //-----------Extended Frame
    uint32_t identifier =  ((uint32_t)CAN_ID_EFF(0)) << 21 ;
             identifier |= ((uint32_t)CAN_ID_EFF(1)) << 13 ;
             identifier |= ((uint32_t)CAN_ID_EFF(2)) << 5  ;
             identifier |= ((uint32_t)CAN_ID_EFF(3)) >> 3  ;
    outFrame.id = identifier;

    for (uint8_t i=0 ; i<outFrame.len ; i++) {
      outFrame.data[i] = CAN_DATA_EFF(i);
    }
  }

  CAN_CMD = CAN_CMD_RELEASE_RXB;
}

//--------------------------------------------------------------------------------------------------
//   TRANSMISSION
//--------------------------------------------------------------------------------------------------

bool ACAN_ESP32::tryToSend (const CANMessage & inMessage) {
  bool sendMessage ;
//--- Bug fixed in 1.0.2 (thanks to DirkMeintjies)
  portENTER_CRITICAL (&mux) ;
    if (mDriverIsSending) {
      sendMessage = mDriverTransmitBuffer.append (inMessage);
    }else{
      internalSendMessage (inMessage) ;
      mDriverIsSending = true;
      sendMessage = true;
    }
  portEXIT_CRITICAL (&mux) ;
  return sendMessage ;
}

//--------------------------------------------------------------------------------------------------

void ACAN_ESP32::internalSendMessage (const CANMessage &inFrame) {
//--- DLC
  const uint8_t dlc = (inFrame.len <= 8) ? inFrame.len : 8;
//--- RTR
  const uint8_t rtr = inFrame.rtr ? CAN_RTR : 0 ;
//--- Frame ID
  const uint8_t id = (inFrame.ext) ? CAN_FRAME_FORMAT_EFF : CAN_FRAME_FORMAT_SFF ;
//--- Set Frame Information
  CAN_FRAME_INFO = id | rtr | CAN_DLC (dlc) ;
//--- Identifier and data
  if (!inFrame.ext) { //-------Standard Frame
  //--- Set ID
    CAN_ID_SFF(0)  = uint8_t (inFrame.id >> 3) ;
    CAN_ID_SFF(1)  = uint8_t (inFrame.id << 5) ;
  //--- Set data
    for (uint8_t i=0 ; i<dlc ; i++) {
      CAN_DATA_SFF (i) = inFrame.data [i];
    }
  }else{ //-------Extended Frame
  //--- Set ID
   CAN_ID_EFF(0) = uint8_t (inFrame.id >> 21);
   CAN_ID_EFF(1) = uint8_t (inFrame.id >> 13);
   CAN_ID_EFF(2) = uint8_t (inFrame.id >> 5);
   CAN_ID_EFF(3) = uint8_t (inFrame.id << 3);
  //--- Set data
    for (uint8_t i=0 ; i<dlc ; i++) {
      CAN_DATA_EFF (i) = inFrame.data [i];
    }
  }
//--- Send command
  CAN_CMD = ((CAN_MODE & CAN_MODE_SELFTEST) != 0) ? CAN_CMD_SELF_RX_REQ : CAN_CMD_TX_REQ ;
}

//--------------------------------------------------------------------------------------------------
//    Driver instance
//--------------------------------------------------------------------------------------------------

ACAN_ESP32 ACAN_ESP32::can ;

//--------------------------------------------------------------------------------------------------
