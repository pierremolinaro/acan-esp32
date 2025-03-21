//------------------------------------------------------------------------------
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
//
//------------------------------------------------------------------------------

#pragma once

//------------------------------------------------------------------------------
//   Include files
//------------------------------------------------------------------------------

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_intr_alloc.h>
#include <soc/gpio_sig_map.h>
#include <soc/periph_defs.h>
#include <soc/interrupts.h>
// /Users/pierremolinaro/Library/Arduino15/packages/esp32/tools/esp32-arduino-libs/idf-release_v5.4-bcb3c32d-v1/esp32c6/include/soc/esp32c6/include/soc/gpio_sig_map.h:138:9: note: it was later defined here

//------------------------------------------------------------------------------

#if defined (CONFIG_IDF_TARGET_ESP32S3)
  static const uint32_t twaiBaseAddress = DR_REG_TWAI_BASE ; // 0x6002B000
  static const uint32_t twaiTxPinSelector = TWAI_TX_IDX ;
  static const uint32_t twaiRxPinSelector = TWAI_RX_IDX ;
  static const periph_module_t twaiPeriphModule = PERIPH_TWAI_MODULE ;
  static const periph_interrput_t twaiInterruptSource = ETS_TWAI_INTR_SOURCE ;
#elif defined (CONFIG_IDF_TARGET_ESP32S2)
  static const uint32_t twaiBaseAddress = 0x3F42B000 ;
  static const uint32_t twaiTxPinSelector = TWAI_TX_IDX ;
  static const uint32_t twaiRxPinSelector = TWAI_RX_IDX ;
  static const periph_module_t twaiPeriphModule = PERIPH_TWAI_MODULE ;
  static const periph_interrput_t twaiInterruptSource = ETS_TWAI_INTR_SOURCE ;
#elif defined (CONFIG_IDF_TARGET_ESP32C3)
  static const uint32_t twaiBaseAddress = DR_REG_TWAI_BASE ; // 0x6002B000
  static const uint32_t twaiTxPinSelector = TWAI_TX_IDX ;
  static const uint32_t twaiRxPinSelector = TWAI_RX_IDX ;
  static const periph_module_t twaiPeriphModule = PERIPH_TWAI_MODULE ;
  static const periph_interrput_t twaiInterruptSource = ETS_TWAI_INTR_SOURCE ;
#elif defined (CONFIG_IDF_TARGET_ESP32C6)
  // twaiBaseAddress, twaiTxPinSelector, twaiRxPinSelector, twaiPeriphModule
  // and twaiInterruptSource are defined as instance properties
  // of ACAN_ESP32 class
#elif defined (CONFIG_IDF_TARGET_ESP32)
  static const uint32_t twaiBaseAddress = DR_REG_CAN_BASE ; // 0x3ff6B000
  static const uint32_t twaiTxPinSelector = TWAI_TX_IDX ;
  static const uint32_t twaiRxPinSelector = TWAI_RX_IDX ;
  static const periph_module_t twaiPeriphModule = PERIPH_TWAI_MODULE ;
  static const periph_interrput_t twaiInterruptSource = ETS_TWAI_INTR_SOURCE ;
#else
  #error "ESP32 TWAI (CAN) module not handled for this platform"
#endif

//------------------------------------------------------------------------------
