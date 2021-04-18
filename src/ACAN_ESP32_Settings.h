/******************************************************************************/
/* File name        : ACAN_ESP32_Settings.h                                   */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN Bit Timing Calculator and Initial settings    */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/
/*  Version |  Date       | Change                                            */
/* ---------------------------------------------------------------------------*/
/*   V1.0   | 25 Apr 2019 | Creation                                          */
/*   V1.1   | 03 May 2019 | Actual BitRate Calculation                        */
/*   V1.2   | 06 May 2019 | Added Error Conditions                            */
/*   V1.3   | 21 May 2019 | Added internal clock source  (APB CLOCK)          */
/*   V1.4   | 04 Jun 2019 | Added CAN operating Mode                          */
/*   V1.5   | 15 Jul 2019 | Added driver buffers                              */
/*   V2.0   | 08 Aug 2019 | Message Control types                             */
/* ---------------------------------------------------------------------------*/

#pragma once

//------------------------------- Include files ----------------------------------------------------

#include <stdint.h>
#include <freertos/FreeRTOS.h>

//--------------------------------------------------------------------------------------------------
//  ESP32 ACANSettings class
//--------------------------------------------------------------------------------------------------

class ACAN_ESP32_Settings {

  //································································································
  //   ENUMERATED TYPES
  //································································································

  //--- CAN driver operating modes
    public: typedef enum : uint8_t {
        NormalMode,
        ListenOnlyMode,
        LoopBackMode
    } CANMode ;

  //································································································
  //   CONSTRUCTOR
  //································································································

  public: ACAN_ESP32_Settings (const uint32_t inDesiredBitRate,
                               const uint32_t inTolerancePPM = 1000);

  //································································································
  //   CAN PINS
  //································································································

  public: gpio_num_t mTxPin = GPIO_NUM_5 ;
  public: gpio_num_t mRxPin = GPIO_NUM_4 ;

  //································································································
  //   CAN BIT TIMING
  //································································································

  public: uint32_t mDesiredBitRate ;                 // In kb/s
  public: uint8_t mBitRatePrescaler = 0 ;            // 2...128
  public: uint8_t mTimeSegment1 = 0 ;                // 1...16
  public: uint8_t mTimeSegment2 = 0 ;                // 1...8
  public: uint8_t mSJW = 0 ;                         // 1...4
  public: bool mTripleSampling = false ;             // true --> triple sampling, false --> single sampling
  public: bool mBitRateClosedToDesiredRate = false ; // The above configuration is correct

  //································································································
  //   Max values
  //································································································

    public: static const uint8_t Sync_Seg            = 1 ;    // Fixed Sync Segment
    public: static const uint8_t MAX_BRP             = 128 ;
    public: static const uint8_t MIN_BRP             = 2 ;
    public: static const uint8_t MAX_TQ              = 25 ;
    public: static const uint8_t MIN_TQ              = 3 ;
    public: static const uint8_t MAX_TIME_SEGMENT_1  = 16 ;
    public: static const uint8_t MAX_TIME_SEGMENT_2  = 8 ;
    public: static const uint8_t MAX_SJW             = 4 ;

  //································································································
  //   Requested mode
  //································································································

    public: CANMode mRequestedCANMode = NormalMode ;

  //································································································
  //    Receive buffer size
  //································································································

    public: uint16_t mDriverReceiveBufferSize = 32 ;

  //································································································
  //    Transmit buffer sizes
  //································································································

    public: uint16_t mDriverTransmitBufferSize = 16 ;

  //································································································
  //    Compute actual bit rate
  //································································································

    public: uint32_t actualBitRate (void) const ;

  //································································································
  //    Exact bit rate ?
  //································································································

    public: bool exactBitRate (void) const ;

  //································································································
  //    Distance between actual bit rate and requested bit rate (in ppm, part-per-million)
  //································································································

    public: uint32_t ppmFromDesiredBitRate (void) const;

  //································································································
  //    Distance of sample point from bit start (in ppc, part-per-cent, denoted by %)
  //································································································

    public: uint32_t samplePointFromBitStart (void) const;

  //································································································
  //    Bit settings are consistent ? (returns 0 if ok)
  //································································································

    public: uint16_t CANBitSettingConsistency (void) const ;

  //································································································
  //    Constants returned by CANBitSettingConsistency
  //································································································

  public: static const uint16_t kBitRatePrescalerIsLowerThan2                = 1 <<  0 ;
  public: static const uint16_t kBitRatePrescalerIsGreaterThan128            = 1 <<  1 ;
  public: static const uint16_t kTimeSegment1IsZero                          = 1 <<  2 ;
  public: static const uint16_t kTimeSegment1IsGreaterThan16                 = 1 <<  3 ;
  public: static const uint16_t kTimeSegment2IsZero                          = 1 <<  4 ;
  public: static const uint16_t kTimeSegment2IsGreaterThan8                  = 1 <<  5 ;
  public: static const uint16_t kTimeSegment1Is1AndTripleSampling            = 1 <<  6 ;
  public: static const uint16_t kSJWIsZero                                   = 1 <<  7 ;
  public: static const uint16_t kSJWIsGreaterThan4                           = 1 <<  8 ;

  //································································································

} ;

//--------------------------------------------------------------------------------------------------
