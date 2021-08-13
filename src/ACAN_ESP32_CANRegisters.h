/******************************************************************************/
/* File name        : ESP32ACANRegisters.h                                    */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN Peripheral Registers                          */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/
/*  Version |  Date       | Change                                            */
/* ---------------------------------------------------------------------------*/
/*   V1.0   | 20 May 2019 | Configuration Registers                           */
/*   V1.1   | 03 Jun 2019 | Added Shared Registers                            */
/*   V1.2   | 24 Jun 2019 | Registers defined as 32-bit                       */
/* ---------------------------------------------------------------------------*/

#pragma once

//------------------------------- Include files ----------------------------------------------------

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
//#include <esp_intr.h>
#include <esp_intr_alloc.h>
#include <soc/dport_reg.h>
#include <driver/periph_ctrl.h>

//--------------------------------------------------------------------------------------------------
//   ESP32 CAN REGISTERS
//--------------------------------------------------------------------------------------------------

static const uint32_t ESP32CAN_BASE = 0x3FF6B000 ;

//--------------------------------------------------------------------------------------------------
// CAN_MODE
//--------------------------------------------------------------------------------------------------

#define CAN_MODE        (*((volatile uint32_t *)(ESP32CAN_BASE)))

// Bit definitions for CAN_MODE
static const uint32_t CAN_MODE_RESET      = 0x01 ;
static const uint32_t CAN_MODE_LISTENONLY = 0x02 ;
static const uint32_t CAN_MODE_SELFTEST   = 0x04 ;
static const uint32_t CAN_MODE_ACCFILTER  = 0x08 ;

//--------------------------------------------------------------------------------------------------
// CAN_CMD
//--------------------------------------------------------------------------------------------------

#define CAN_CMD         (*((volatile uint32_t *)(ESP32CAN_BASE + 0x004)))

// Bit definitions for CAN_COMMAND
static const uint32_t CAN_CMD_TX_REQ             = 0x01 ;
static const uint32_t CAN_CMD_ABORT_TX           = 0x02 ;
static const uint32_t CAN_CMD_RELEASE_RXB        = 0x04 ;
static const uint32_t CAN_CMD_CLEAR_DATAOVERRUN  = 0x08 ;
static const uint32_t CAN_CMD_SELF_RX_REQ        = 0x10 ;

//--------------------------------------------------------------------------------------------------
// CAN_STATUS
//--------------------------------------------------------------------------------------------------

#define CAN_STATUS      (*((const volatile uint32_t *)(ESP32CAN_BASE + 0x008)))

// Bit definitions for CAN_STATUS
static const uint32_t CAN_STATUS_RXB           = 0x01 ;
static const uint32_t CAN_STATUS_DATAOVERRUN   = 0x02 ;
static const uint32_t CAN_STATUS_TXB           = 0x04 ;
static const uint32_t CAN_STATUS_TX_COMPLETE   = 0x08 ;
static const uint32_t CAN_STATUS_RX            = 0x10 ;
static const uint32_t CAN_STATUS_TX            = 0x20 ;
static const uint32_t CAN_STATUS_ERR           = 0x40 ;
static const uint32_t CAN_STATUS_BUS           = 0x80 ;

//--------------------------------------------------------------------------------------------------
// CAN_INTERRUPT
//--------------------------------------------------------------------------------------------------

#define CAN_INTERRUPT   (*((const volatile uint32_t *)(ESP32CAN_BASE + 0x00C)))

// Bit definitions for CAN_INTERRUPT
static const uint32_t CAN_INTERRUPT_RX           = 0x01;
static const uint32_t CAN_INTERRUPT_TX           = 0x02;
static const uint32_t CAN_INTERRUPT_ERR_WARN     = 0x04;
static const uint32_t CAN_INTERRUPT_DATAOVERRUN  = 0x08;
static const uint32_t CAN_INTERRUPT_ERR_PASSIVE  = 0x20;
static const uint32_t CAN_INTERRUPT_ARB_LOST     = 0x40;
static const uint32_t CAN_INTERRUPT_BUS_ERR      = 0x80;

//--------------------------------------------------------------------------------------------------
// CAN_IER
//--------------------------------------------------------------------------------------------------

#define CAN_IER         (*((volatile uint32_t *)(ESP32CAN_BASE + 0x010)))

static const uint32_t CAN_INTERRUPT_RX_ENABLE = 0x01 ;
static const uint32_t CAN_INTERRUPT_TX_ENABLE = 0x02 ;

//--------------------------------------------------------------------------------------------------
// CAN_BTR0
//--------------------------------------------------------------------------------------------------

  #define CAN_BTR0        (*((volatile uint32_t *)(ESP32CAN_BASE + 0x018)))

//--------------------------------------------------------------------------------------------------
// CAN_BTR1
//--------------------------------------------------------------------------------------------------

#define CAN_BTR1        (*((volatile uint32_t *)(ESP32CAN_BASE + 0x01C)))

//--------------------------------------------------------------------------------------------------
// CAN_ALC
//--------------------------------------------------------------------------------------------------

#define CAN_ALC         (*((volatile uint32_t *)(ESP32CAN_BASE + 0x02C)))

//--------------------------------------------------------------------------------------------------
// CAN_ECC
//--------------------------------------------------------------------------------------------------

#define CAN_ECC         (*((volatile uint32_t *)(ESP32CAN_BASE + 0x030)))

//--------------------------------------------------------------------------------------------------
// CAN_EWLR
//--------------------------------------------------------------------------------------------------

#define CAN_EWLR        (*((volatile uint32_t *)(ESP32CAN_BASE + 0x034)))

//--------------------------------------------------------------------------------------------------
// CAN_RX_ECR
//--------------------------------------------------------------------------------------------------

#define CAN_RX_ECR      (*((volatile uint32_t *)(ESP32CAN_BASE + 0x038)))

//--------------------------------------------------------------------------------------------------
// CAN_TX_ECR
//--------------------------------------------------------------------------------------------------

#define CAN_TX_ECR      (*((volatile uint32_t *)(ESP32CAN_BASE + 0x03C)))

//--------------------------------------------------------------------------------------------------
// CAN_FRAME_INFO
//--------------------------------------------------------------------------------------------------

    //-----CAN Frame Information Register
#define CAN_FRAME_INFO (*((volatile uint32_t *)(ESP32CAN_BASE + 0x040)))

    /* Bit definitions and macros for CAN_TX_RX_FRAME */
static const uint32_t CAN_FRAME_FORMAT_SFF = 0x00;
static const uint32_t CAN_FRAME_FORMAT_EFF = 0x80;
static const uint32_t CAN_RTR              = 0x40;

#define CAN_DLC(x) ((uint8_t(x)) << 0)

//--------------------------------------------------------------------------------------------------
// CAN FRAME REGISTERS
//--------------------------------------------------------------------------------------------------

    //----- SFF : Standard Frame Format - length [2]
    //----- EFF : Extended Frame Format - length [4]
#define CAN_ID_SFF(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x044 + 4 * (idx))))
#define CAN_ID_EFF(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x044 + 4 * (idx))))

#define CAN_MSG_STD_ID 0x7FF
#define CAN_MSG_EXT_ID 0x1FFFFFFF

    //-----CAN Frame Data Register
    //----- DATA : length [8]
#define CAN_DATA_SFF(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x04C + 4 * (idx))))
#define CAN_DATA_EFF(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x054 + 4 * (idx))))

//--------------------------------------------------------------------------------------------------
// CAN Acceptance Filter Registers
//--------------------------------------------------------------------------------------------------

    //----- CODE : length [4]
    //----- MASK : length [4]
#define CAN_ACC_CODE_FILTER(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x040 + 4 * (idx))))
#define CAN_ACC_MASK_FILTER(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x050 + 4 * (idx))))

//--------------------------------------------------------------------------------------------------
// CAN_RXM_COUNTER
//--------------------------------------------------------------------------------------------------

#define CAN_RXM_COUNTER (*((volatile uint32_t *)(ESP32CAN_BASE + 0x074)))

//--------------------------------------------------------------------------------------------------
// CAN_CLK_DIVIDER
//--------------------------------------------------------------------------------------------------

#define CAN_CLK_DIVIDER (*((volatile uint32_t *)(ESP32CAN_BASE + 0x07C)))

static const uint32_t CAN_PELICAN_MODE = 0x80 ;
static const uint32_t CAN_CLK_OFF      = 0x08 ;
#define CAN_CLK_DIV(idx)           ((uint8_t(idx)) << 0)

//--------------------------------------------------------------------------------------------------
// For Accessing ALL ESP32 CAN Registers
//--------------------------------------------------------------------------------------------------

#define REGALL(idx) (*((volatile uint32_t *)(ESP32CAN_BASE + 0x000 + 4 *(idx))))

//--------------------------------------------------------------------------------------------------
