/******************************************************************************/
/* File name        : ESP32CANRegisterTest.ino                                */
/* Project          : ESP32-CAN-DRIVER                                        */
/* Description      : ESP32 CAN REGISTERS ACCESS TEST                         */
/* Compiler         : Arduino IDE                                             */
/* ---------------------------------------------------------------------------*/
/* Copyright        : Copyright © 2019 Pierre Molinaro. All rights reserved.  */
/* ---------------------------------------------------------------------------*/
/* Author           : Mohamed Irfanulla                                       */
/* Supervisor       : Prof. Pierre Molinaro                                   */
/* Institution      : Ecole Centrale de Nantes                                */
/* ---------------------------------------------------------------------------*/


//------------------------------- Board Check ----------------------------------

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//------------------------------- Include files --------------------------------

#include <ACAN_ESP32.h>
#include <ACAN_ESP32_CANRegisters.h>

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup() {
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
  delay (100) ;
//--- Enable the ESP32 CAN Peripheral to access the registers
  periph_module_enable (PERIPH_CAN_MODULE) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

  Serial.println ("ESP32 CAN REGISTERS") ;
  for (uint32_t idx=0; idx<=31 ; idx++) {
    uint32_t value = REGALL(idx);
    uint32_t Reg_no = 0x3FF6B000 + 0x000 + 4 * idx ;
    Serial.printf("REG : %X --- %X \n",Reg_no,value);
  }
}

//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0 ;

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

void loop() {
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

//--- Write Access to bus timming registers
  for (uint16_t i = 0; i <= 255; i++) {
    CAN_BTR0 = byte (i) ;
    CAN_BTR1 = byte (i) ;
    const uint32_t value1 = CAN_BTR0 ;
    const uint32_t value2 = CAN_BTR1 ;
    //      Serial.printf ("Value : %d---Reg BTR0 : %X \n",i,value1);
    //      Serial.printf ("Value : %d---Reg BTR1 : %X \n",i,value2);
    if ((i != value1) && (i != value2)) {
      Serial.println ("reg error") ;
    }else {
      Serial.println ("Register written successful") ;
      delay (1000) ;
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
