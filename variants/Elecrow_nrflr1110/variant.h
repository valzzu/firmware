/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.d
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ELECROW_NRFLR1110_
#define _VARIANT_ELECROW_NRFLR1110_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul) //elecrow???

// #define USE_LFXO // Board uses 32khz crystal for LF
#define USE_LFRC // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// Use the native nrf52 usb power detection
#define NRF_APM

//#define PIN_3V3_EN (32 + 6)     // P1.6, Power to Sensors
//#define PIN_3V3_ACC_EN (32 + 7) // P1.7, Power to Acc

// LED
#define PIN_LED1 (0 + 15) // P0.15
#define LED_BUILTIN PIN_LED1
// Actually red
#define LED_BLUE PIN_LED1
#define LED_STATE_ON 1 // State when LED is lit


// Button
#define BUTTON_PIN (32 + 0) // P1.00

#define HAS_WIRE 1

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (32 + 4)      // P1.04
#define PIN_WIRE_SCL (32 + 2)      // P1.27

/*
 * Serial interfaces
 */

 // GPS
#define PIN_GPS_TX (0 + 22) // P0.22
#define PIN_GPS_RX (0 + 20) // P0.20

#define HAS_GPS 1
#define PIN_GPS_EN (0 + 24) // P0.24
// define GPS_DEBUG

#define PIN_SERIAL1_RX PIN_GPS_RX 
#define PIN_SERIAL1_TX PIN_GPS_TX 

// not used on the board but doesent compile without these
#define PIN_SERIAL2_RX (32 + 9) // P1.09 
#define PIN_SERIAL2_TX (0 + 8) // P0.08

#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO (32 + 15) // P1.15
#define PIN_SPI_MOSI (32 + 14) // P1.14
#define PIN_SPI_SCK (32 + 13)  // P1.13
#define PIN_SPI_NSS (32 + 12)  // P1.12


// #define HAS_SDCARD
#define PIN_SPI1_MOSI (0 + 2) //P0.3
#define PIN_SPI1_SCK (0 + 28) //P0.28
#define PIN_SPI1_MISO (0 + 30)  //P0.30
#define PIN_SPI1_CS (0 + 2) //P0.02
#define SDCARD_CS PIN_SPI_CS
//#define SD_SPI_FREQUENCY 75000000U 

#define PIN_SPI_MISO (32 + 15)
#define PIN_SPI_MOSI (32 + 14)
#define PIN_SPI_SCK  (32 + 13)
#define PIN_SPI_NSS  (32 + 12)

#define LORA_RESET (32 + 10)
#define LORA_DIO1 (40)
#define LORA_DIO2 (43)
#define LORA_SCK PIN_SPI_SCK
#define LORA_MISO PIN_SPI_MISO
#define LORA_MOSI PIN_SPI_MOSI
#define LORA_CS PIN_SPI_NSS

// supported modules list
#define USE_LR1110

#define LR1110_IRQ_PIN LORA_DIO1
#define LR1110_NRESET_PIN LORA_RESET
#define LR1110_BUSY_PIN LORA_DIO2
#define LR1110_SPI_NSS_PIN LORA_CS
#define LR1110_SPI_SCK_PIN LORA_SCK
#define LR1110_SPI_MOSI_PIN LORA_MOSI
#define LR1110_SPI_MISO_PIN LORA_MISO

#define LR11X0_DIO3_TCXO_VOLTAGE 1.6
#define LR11X0_DIO_AS_RF_SWITCH

#define SX126X_TXEN (0 + 17) // P0.17

#define BATTERY_PIN (0 + 31)           // P0.31
#define BATTERY_IMMUTABLE
#define ADC_MULTIPLIER (2.0F)

#define ADC_RESOLUTION 14
#define BATTERY_SENSE_RESOLUTION_BITS 12

#undef AREF_VOLTAGE
#define AREF_VOLTAGE 3.0
#define VBAT_AR_INTERNAL AR_INTERNAL_3_0

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif // _VARIANT_TRACKER_T1000_E_
