
#pragma once

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------

#include <stdint.h>
//#include <driver/periph_ctrl.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_intr_alloc.h>

//----------------------------------------------------------------------------------------
//   ESP32 TWAI REGISTER BASE
//   See sdkconfig.h files for defining CONFIG_IDF_TARGET_xxxx
//   DR_REG_TWAI_BASE is defined in:
//     - ~/Library/Arduino15/packages/esp32/hardware/esp32/2.0.11/tools/sdk/esp32c3/include/soc/esp32c3/include/soc/soc.h
//     - ~//Library/Arduino15/packages/esp32/hardware/esp32/2.0.11/tools/sdk/esp32s3/include/soc/esp32s3/include/soc/soc.h
//   DR_REG_CAN_BASE is defined in:
//     - ~//Library/Arduino15/packages/esp32/hardware/esp32/2.0.11/tools/sdk/esp32/include/soc/esp32/include/soc/soc.h
//  No definition for ESP32-S2 in ~/Library/Arduino15/packages/esp32/hardware/esp32/2.0.11/tools/sdk/esp32s2/include/soc/esp32s2/include/soc/soc.h
//  However the ~/Library/Arduino15/packages/esp32/hardware/esp32/2.0.11/tools/ide-debug/svd/esp32s2.svd file defines
//  the TWAI base address: 0x3F42B000
//  But ESP32S2 reference manual (§3.3.5) gives two addresses:
//     - 0x3F40_0000 + 0x0002_B000 = 0x3F42_B000 from PeriBus1 (faster)
//     - 0x6000_0000 + 0x0002_B000 = 0x6002_B000 from PeriBus2

//----------------------------------------------------------------------------------------

#if defined (CONFIG_IDF_TARGET_ESP32S3)
  static const uint32_t ESP32_TWAI_BASE = DR_REG_TWAI_BASE ; // 0x6002B000
#elif defined (CONFIG_IDF_TARGET_ESP32S2)
  static const uint32_t ESP32_TWAI_BASE = 0x3F42B000 ;
#elif defined (CONFIG_IDF_TARGET_ESP32C3)
  static const uint32_t ESP32_TWAI_BASE = DR_REG_TWAI_BASE ; // 0x6002B000
#elif defined (CONFIG_IDF_TARGET_ESP32)
  static const uint32_t ESP32_TWAI_BASE = DR_REG_CAN_BASE ; // 0x3ff6B000
#else
  #error "ESP32 TWAI (CAN) module not handled for this platform"
#endif

//----------------------------------------------------------------------------------------
// TWAI_MODE_REG
//----------------------------------------------------------------------------------------

#define TWAI_MODE_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE)))

// Bit definitions for TWAI_MODE_REG
static const uint32_t TWAI_RESET_MODE       = 0x01 ;
static const uint32_t TWAI_LISTEN_ONLY_MODE = 0x02 ;
static const uint32_t TWAI_SELF_TEST_MODE   = 0x04 ;
static const uint32_t TWAI_RX_FILTER_MODE   = 0x08 ;

//----------------------------------------------------------------------------------------
// TWAI_CMD
//----------------------------------------------------------------------------------------

#define TWAI_CMD_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x004)))

// Bit definitions for TWAI_CMD_REG
static const uint32_t TWAI_TX_REQ      = 0x01 ;
static const uint32_t TWAI_ABORT_TX    = 0x02 ;
static const uint32_t TWAI_RELEASE_BUF = 0x04 ;
static const uint32_t TWAI_CLR_OVERRUN = 0x08 ;
static const uint32_t TWAI_SELF_RX_REQ = 0x10 ;

//----------------------------------------------------------------------------------------
// TWAI_STATUS_REG
//----------------------------------------------------------------------------------------

#define TWAI_STATUS_REG (*((const volatile uint32_t *)(ESP32_TWAI_BASE + 0x008)))

// Bit definitions for TWAI_STATUS_REG
static const uint32_t TWAI_RX_BUF_ST   = 0x01 ;
static const uint32_t TWAI_OVERRUN_ST  = 0x02 ;
static const uint32_t TWAI_TX_BUF_ST   = 0x04 ;
static const uint32_t TWAI_TX_COMPLETE = 0x08 ;
static const uint32_t TWAI_RX_ST       = 0x10 ;
static const uint32_t TWAI_TX_ST       = 0x20 ;
static const uint32_t TWAI_ERR_ST      = 0x40 ;
static const uint32_t TWAI_BUS_OFF_ST  = 0x80 ;

//----------------------------------------------------------------------------------------
// TWAI_INT_RAW_REG
//----------------------------------------------------------------------------------------

#define TWAI_INT_RAW_REG (*((const volatile uint32_t *)(ESP32_TWAI_BASE + 0x00C)))

// Bit definitions for TWAI_INT_RAW_REG
static const uint32_t TWAI_RX_INT_ST          = 0x01;
static const uint32_t TWAI_TX_INT_ST          = 0x02;
static const uint32_t TWAI_ERR_WARN_INT_ST    = 0x04;
static const uint32_t TWAI_OVERRUN_INT_ST     = 0x08;
static const uint32_t TWAI_ERR_PASSIVE_INT_ST = 0x20;
static const uint32_t TWAI_ARB_LOST_INT_ST    = 0x40;
static const uint32_t TWAI_BUS_ERR_INT_ST     = 0x80;

//----------------------------------------------------------------------------------------
// TWAI_INT_ENA_REG
//----------------------------------------------------------------------------------------

#define TWAI_INT_ENA_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x010)))

static const uint32_t TWAI_RX_INT_ENA = 0x01 ;
static const uint32_t TWAI_TX_INT_ENA = 0x02 ;

//----------------------------------------------------------------------------------------
// TWAI_BUS_TIMING_0_REG
//----------------------------------------------------------------------------------------

#define TWAI_BUS_TIMING_0_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x018)))

//----------------------------------------------------------------------------------------
// TWAI_BUS_TIMING_1_REG
//----------------------------------------------------------------------------------------

#define TWAI_BUS_TIMING_1_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x01C)))

//----------------------------------------------------------------------------------------
// TWAI_ARB_LOST_CAP_REG
//----------------------------------------------------------------------------------------

#define TWAI_ARB_LOST_CAP_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x02C)))

//----------------------------------------------------------------------------------------
// TWAI_ERR_CODE_CAP_REG
//----------------------------------------------------------------------------------------

#define TWAI_ERR_CODE_CAP_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x030)))

//----------------------------------------------------------------------------------------
// TWAI_ERR_WARNING_LIMIT_REG
//----------------------------------------------------------------------------------------

#define TWAI_ERR_WARNING_LIMIT_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x034)))

//----------------------------------------------------------------------------------------
// TWAI_RX_ERR_CNT_REG
//----------------------------------------------------------------------------------------

#define TWAI_RX_ERR_CNT_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x038)))

//----------------------------------------------------------------------------------------
// TWAI_TX_ERR_CNT_REG
//----------------------------------------------------------------------------------------

#define TWAI_TX_ERR_CNT_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x03C)))

//----------------------------------------------------------------------------------------
// TWAI_FRAME_INFO
//----------------------------------------------------------------------------------------

    //-----CAN Frame Information Register
#define TWAI_FRAME_INFO (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x040)))

    /* Bit definitions and macros for TWAI_TX_RX_FRAME */
static const uint32_t TWAI_FRAME_FORMAT_SFF = 0x00;
static const uint32_t TWAI_FRAME_FORMAT_EFF = 0x80;
static const uint32_t TWAI_RTR              = 0x40;

#define TWAI_DLC(x) ((uint8_t(x)) << 0)

//----------------------------------------------------------------------------------------
// CAN FRAME REGISTERS
//----------------------------------------------------------------------------------------

    //----- SFF : Standard Frame Format - length [2]
    //----- EFF : Extended Frame Format - length [4]
#define TWAI_ID_SFF(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x044 + 4 * (idx))))
#define TWAI_ID_EFF(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x044 + 4 * (idx))))

#define TWAI_MSG_STD_ID (0x7FF)
#define TWAI_MSG_EXT_ID (0x1FFFFFFF)

    //-----CAN Frame Data Register
    //----- DATA : length [8]
#define TWAI_DATA_SFF(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x04C + 4 * (idx))))
#define TWAI_DATA_EFF(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x054 + 4 * (idx))))

//----------------------------------------------------------------------------------------
// CAN Acceptance Filter Registers
//----------------------------------------------------------------------------------------

    //----- CODE : length [4]
    //----- MASK : length [4]
#define TWAI_ACC_CODE_FILTER(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x040 + 4 * (idx))))
#define TWAI_ACC_MASK_FILTER(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x050 + 4 * (idx))))

//----------------------------------------------------------------------------------------
// TWAI_RX_MESSAGE_COUNTER_REG
//----------------------------------------------------------------------------------------

#define TWAI_RX_MESSAGE_COUNTER_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x074)))

//----------------------------------------------------------------------------------------
// TWAI_CLOCK_DIVIDER_REG
//----------------------------------------------------------------------------------------

#define TWAI_CLOCK_DIVIDER_REG (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x07C)))

// Bit definitions for TWAI_MODE_REG
static const uint32_t TWAI_EXT_MODE = 0x80 ;

//----------------------------------------------------------------------------------------
// For Accessing ALL ESP32 CAN Registers
//----------------------------------------------------------------------------------------

#define REGALL(idx) (*((volatile uint32_t *)(ESP32_TWAI_BASE + 0x000 + 4 *(idx))))

//----------------------------------------------------------------------------------------
