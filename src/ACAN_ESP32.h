/******************************************************************************/
/* File name        : ACAN_ESP32.h                                            */
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

#pragma once

//------------------------------- Include files ----------------------------------------------------

#include <ACAN_ESP32_CANRegisters.h>
#include <ACAN_ESP32_Settings.h>
#include <CANMessage.h>
#include <ACAN_ESP32_Buffer16.h>
#include <ACAN_ESP32_AcceptanceFilters.h>

//--------------------------------------------------------------------------------------------------
//   ESP32 CAN class
//--------------------------------------------------------------------------------------------------

class ACAN_ESP32 {

  //································································································
  //   CONSTRUCTOR
  //································································································

  private: ACAN_ESP32 (void) ;

  //································································································
  //    Initialisation: returns 0 if ok, otherwise see error codes below
  //································································································

  public: uint32_t begin (const ACAN_ESP32_Settings & inSettings,
                          const ACAN_ESP32_Filter & inFilterSettings = ACAN_ESP32_Filter::acceptAll ()) ;


  //································································································
  //    CAN  Configuration Private Methods
  //································································································

  private: void setGPIOPins (const gpio_num_t inTXPin, const gpio_num_t inRXPin);
  private: void setBitTimingSettings(const ACAN_ESP32_Settings &inSettings) ;
  private: void setRequestedCANMode (const ACAN_ESP32_Settings &inSettings,
                                     const ACAN_ESP32_Filter & inFilter) ;
  private: void setAcceptanceFilter (const ACAN_ESP32_Filter & inFilter) ;

  //································································································
  //    Receiving messages
  //································································································

  public: bool receive (CANMessage &outMessage) ;
  public: static void getReceivedMessage (CANMessage & outFrame) ;

  //································································································
  //    Receive buffer
  //································································································

  private: ACAN_ESP32_Filter::Format mAcceptedFrameFormat ;

  private: ACAN_ESP32_Buffer16 mDriverReceiveBuffer ;

  public: inline uint16_t driverReceiveBufferSize (void) const { return mDriverReceiveBuffer.size () ;  }
  public: inline uint16_t driverReceiveBufferCount (void) const { return mDriverReceiveBuffer.count() ;  }
  public: inline uint16_t driverReceiveBufferPeakCount (void) const { return mDriverReceiveBuffer.peakCount () ; }

  public: inline void resetDriverReceiveBufferPeakCount (void) { mDriverReceiveBuffer.resetPeakCount () ; }

  //································································································
  //    Transmitting messages
  //································································································

  public: bool tryToSend (const CANMessage & inMessage) ;
  private: void internalSendMessage (const CANMessage & inFrame) ;

  //································································································
  //    Transmit buffer
  //································································································

  private: ACAN_ESP32_Buffer16 mDriverTransmitBuffer ;
  private: bool mDriverIsSending ;

  public: inline uint16_t driverTransmitBufferSize (void) const { return mDriverTransmitBuffer.size () ; }
  public: inline uint16_t driverTransmitBufferCount (void) const { return mDriverTransmitBuffer.count () ; }
  public: inline uint16_t driverTransmitBufferPeakCount (void) const { return mDriverTransmitBuffer.peakCount () ; }

  public: inline void resetDriverTransmitBufferPeakCount (void) { mDriverTransmitBuffer.resetPeakCount () ; }

  //································································································
  //    Error codes returned by begin
  //································································································

  public: static const uint32_t kNotInRestModeInConfiguration       = 1 << 16 ;
  public: static const uint32_t kCANRegistersError                  = 1 << 17 ;
  public: static const uint32_t kTooFarFromDesiredBitRate           = 1 << 18 ;
  public: static const uint32_t kInconsistentBitRateSettings        = 1 << 19 ;
  public: static const uint32_t kCannotAllocateDriverReceiveBuffer  = 1 << 20 ;
  public: static const uint32_t kCannotAllocateDriverTransmitBuffer = 1 << 21 ;

  //································································································
  //    Interrupt Handler
  //································································································

  public: static void IRAM_ATTR isr (void * inUserArgument) ;

  public: void handleTXInterrupt (void) ;
  public: void handleRXInterrupt (void) ;

  //································································································
  //    No Copy
  //································································································

  private: ACAN_ESP32 (const ACAN_ESP32 &) = delete ;
  private: ACAN_ESP32 & operator = (const ACAN_ESP32 &) = delete ;

  //································································································
  //    Driver instance
  //································································································

  public: static ACAN_ESP32 can ;

  //································································································

} ;

//--------------------------------------------------------------------------------------------------
