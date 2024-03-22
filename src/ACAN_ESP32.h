
#pragma once

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <ACAN_ESP32_CANRegisters.h>
#include <ACAN_ESP32_Settings.h>
#include <ACAN_ESP32_CANMessage.h>
#include <ACAN_ESP32_Buffer16.h>
#include <ACAN_ESP32_AcceptanceFilters.h>

//----------------------------------------------------------------------------------------
//   ESP32 CAN class
//----------------------------------------------------------------------------------------

class ACAN_ESP32 {

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   CONSTRUCTOR
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACAN_ESP32 (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Initialisation: returns 0 if ok, otherwise see error codes below
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: uint32_t begin (const ACAN_ESP32_Settings & inSettings,
                          const ACAN_ESP32_Filter & inFilterSettings = ACAN_ESP32_Filter::acceptAll ()) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    CAN  Configuration Private Methods
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: void setGPIOPins (const gpio_num_t inTXPin, const gpio_num_t inRXPin);
  private: void setBitTimingSettings(const ACAN_ESP32_Settings &inSettings) ;
  private: void setRequestedCANMode (const ACAN_ESP32_Settings &inSettings,
                                     const ACAN_ESP32_Filter & inFilter) ;
  private: void setAcceptanceFilter (const ACAN_ESP32_Filter & inFilter) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Receiving messages
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool available (void) const ;
  public: bool receive (CANMessage & outMessage) ;
  public: static void getReceivedMessage (CANMessage & outFrame) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Receive buffer
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACAN_ESP32_Filter::Format mAcceptedFrameFormat ;

  private: ACAN_ESP32_Buffer16 mDriverReceiveBuffer ;

  public: inline uint16_t driverReceiveBufferSize (void) const { return mDriverReceiveBuffer.size () ;  }
  public: inline uint16_t driverReceiveBufferCount (void) const { return mDriverReceiveBuffer.count() ;  }
  public: inline uint16_t driverReceiveBufferPeakCount (void) const { return mDriverReceiveBuffer.peakCount () ; }

  public: inline void resetDriverReceiveBufferPeakCount (void) { mDriverReceiveBuffer.resetPeakCount () ; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Transmitting messages
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool tryToSend (const CANMessage & inMessage) ;
  private: void internalSendMessage (const CANMessage & inFrame) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Transmit buffer
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACAN_ESP32_Buffer16 mDriverTransmitBuffer ;
  private: bool mDriverIsSending ;

  public: inline uint16_t driverTransmitBufferSize (void) const { return mDriverTransmitBuffer.size () ; }
  public: inline uint16_t driverTransmitBufferCount (void) const { return mDriverTransmitBuffer.count () ; }
  public: inline uint16_t driverTransmitBufferPeakCount (void) const { return mDriverTransmitBuffer.peakCount () ; }

  public: inline void resetDriverTransmitBufferPeakCount (void) { mDriverTransmitBuffer.resetPeakCount () ; }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Error codes returned by begin
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: static const uint32_t kNotInResetModeInConfiguration      = 1 << 16 ;
  public: static const uint32_t kCANRegistersError                  = 1 << 17 ;
  public: static const uint32_t kTooFarFromDesiredBitRate           = 1 << 18 ;
  public: static const uint32_t kInconsistentBitRateSettings        = 1 << 19 ;
  public: static const uint32_t kCannotAllocateDriverReceiveBuffer  = 1 << 20 ;
  public: static const uint32_t kCannotAllocateDriverTransmitBuffer = 1 << 21 ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Interrupt Handler
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: static void IRAM_ATTR isr (void * inUserArgument) ;
  private: intr_handle_t mInterruptHandler ;

  public: void handleTXInterrupt (void) ;
  public: void handleRXInterrupt (void) ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // STATUS FLAGS
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  //--- Status Flags (returns 0 if no error)
  //  Bit 0 : hardware receive FIFO overflow
  //  Bit 1 : driver receive FIFO overflow
  //  Bit 2 : bus off
  //  Bit 3 : reset mode

  public: uint32_t statusFlags (void) const ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Recover from Bus-Off
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: bool recoverFromBusOff (void) const ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    No Copy
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  private: ACAN_ESP32 (const ACAN_ESP32 &) = delete ;
  private: ACAN_ESP32 & operator = (const ACAN_ESP32 &) = delete ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //    Driver instance
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  public: static ACAN_ESP32 can ;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

} ;

//----------------------------------------------------------------------------------------
