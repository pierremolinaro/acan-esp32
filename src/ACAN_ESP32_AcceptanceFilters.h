/******************************************************************************/
/* File name        : ESP32AcceptanceFilters.h                                */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN Acceptance Filter settings                    */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/
/*  Version | Change                                                          */
/* ---------------------------------------------------------------------------*/
/*   V1.0   | Creation default filter                                         */
/*   V1.1   | Single Filter settings                                          */
/*   V1.2   | Dual Filter settings                                            */
/* ---------------------------------------------------------------------------*/

#pragma once

//--------------------------------------------------------------------------------------------------

#include <Arduino.h>

//--------------------------------------------------------------------------------------------------

class ACAN_ESP32_Filter {

  public: ACAN_ESP32_Filter (void) :
  mACR0 (0),
  mACR1 (0),
  mACR2 (0),
  mACR3 (0),
  mAMR0 (0),
  mAMR1 (0),
  mAMR2 (0),
  mAMR3 (0),
  mAMFSingle (false) {
  }

  public: uint8_t mACR0 ;
  public: uint8_t mACR1 ;
  public: uint8_t mACR2 ;
  public: uint8_t mACR3 ;
  public: uint8_t mAMR0 ;
  public: uint8_t mAMR1 ;
  public: uint8_t mAMR2 ;
  public: uint8_t mAMR3 ;
  public: bool mAMFSingle ;
} ;

//--------------------------------------------------------------------------------------------------

inline ACAN_ESP32_Filter acceptAllFilter (void) {
  ACAN_ESP32_Filter result;
  result.mACR0 = 0x00;
  result.mACR1 = 0x00;
  result.mACR2 = 0x00;
  result.mACR3 = 0x00;
  result.mAMR0 = 0xFF;
  result.mAMR1 = 0xFF;
  result.mAMR2 = 0xFF;
  result.mAMR3 = 0xFF;
  return result;
}

//--------------------------------------------------------------------------------------------------

inline ACAN_ESP32_Filter acceptSingleFilterStandard (const uint16_t inCodeID,
                                                     const uint8_t inCByte0,
                                                    const uint8_t inCByte1,
                                                    const uint16_t inMaskID,
                                                    const uint8_t inMByte0,
                                                    const uint8_t inMByte1) {
  ACAN_ESP32_Filter result ;
  result.mAMFSingle = true; // Single Filter
  result.mACR0 = uint8_t (inCodeID >> 3);
  result.mACR1 = uint8_t (inCodeID << 5);
  result.mACR2 = inCByte0;
  result.mACR3 = inCByte1;

  result.mAMR0 = uint8_t (inMaskID >> 3);
  result.mAMR1 = uint8_t (inMaskID << 5) | 0x1F;
  result.mAMR2 = inMByte0;
  result.mAMR3 = inMByte1;
  return result;
}

//--------------------------------------------------------------------------------------------------

inline ACAN_ESP32_Filter acceptSingleFilterExtended (const uint32_t inCodeID,
                                                     const uint32_t /* inMaskID */) {
  ACAN_ESP32_Filter result;
  result.mAMFSingle = true; // Single Filter
  result.mACR0 = uint8_t (inCodeID >> 21);
  result.mACR1 = uint8_t (inCodeID >> 13);
  result.mACR2 = uint8_t (inCodeID >> 5);
  result.mACR3 = uint8_t (inCodeID << 3);

  result.mAMR0 = uint8_t (inCodeID >> 21);
  result.mAMR1 = uint8_t (inCodeID >> 13);
  result.mAMR2 = uint8_t (inCodeID >> 5);
  result.mAMR3 = uint8_t (inCodeID << 3);
  return result;
}

//--------------------------------------------------------------------------------------------------

inline ACAN_ESP32_Filter acceptDualFilterStandard (const uint16_t inFilter1CodeID,
                                                   const uint16_t inFilter2CodeID,
                                                   const uint8_t inCByte0,
                                                   const uint16_t inFilter1MaskID,
                                                   const uint16_t inFilter2MaskID,
                                                   const uint8_t inMByte0) {
  ACAN_ESP32_Filter result;
  result.mACR0 = uint8_t (inFilter1CodeID >> 3) ;
  result.mACR1 = uint8_t ((inFilter1CodeID << 5) | (inCByte0 >> 4)) ;
  result.mACR2 = uint8_t (inFilter2CodeID >> 3) ;
  result.mACR3 = uint8_t ((inFilter2CodeID << 5) | (inCByte0 << 0)) ;

  result.mAMR0 = uint8_t (inFilter1MaskID >> 3);
  result.mAMR1 = uint8_t ((inFilter1MaskID << 5) | (inMByte0 >> 4)) | 0x1F;
  result.mAMR2 = uint8_t (inFilter2MaskID >> 3);
  result.mAMR3 = uint8_t ((inFilter2MaskID << 5) | (inMByte0 << 0)) | 0x1F;

  return result;
}

//--------------------------------------------------------------------------------------------------

inline ACAN_ESP32_Filter acceptDualFilterExtended (const uint32_t inFilter1CodeID,
                                                   const uint32_t inFilter2CodeID,
                                                   const uint32_t inFilter1MaskID,
                                                   const uint32_t inFilter2MaskID) {
  ACAN_ESP32_Filter result;
  result.mACR0 = uint8_t (inFilter1CodeID >> 21);
  result.mACR1 = uint8_t (inFilter1CodeID >> 13);
  result.mACR2 = uint8_t (inFilter2CodeID >> 21);
  result.mACR3 = uint8_t (inFilter2CodeID >> 13);

  result.mAMR0 = uint8_t (inFilter1MaskID >> 21);
  result.mAMR1 = uint8_t (inFilter1MaskID >> 13);
  result.mAMR2 = uint8_t (inFilter2MaskID >> 21);
  result.mAMR3 = uint8_t (inFilter2MaskID >> 13);
  return result;
}

//--------------------------------------------------------------------------------------------------
