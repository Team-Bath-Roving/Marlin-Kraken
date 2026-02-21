/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "env_validate.h"

#define BOARD_INFO_NAME      "KRAKEN V1.0"

#define USES_DIAG_JUMPERS

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #undef NO_EEPROM_SELECTED
  #ifndef FLASH_EEPROM_EMULATION
    #define FLASH_EEPROM_EMULATION
  #endif
  #define EEPROM_PAGE_SIZE      (0x800UL) // 2K
  #define EEPROM_START_ADDRESS  (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PE9

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC15  // M1-STOP, MIN1
#define Y_DIAG_PIN                          PF0   // M2-STOP, MIN2
#define Z_DIAG_PIN                          PF1   // M3-STOP, MIN3
#define Z2_DIAG_PIN                         PF2   // M4-STOP, MIN4
#define E0_DIAG_PIN                         PF3   // M5-STOP, MIN5
#define E1_DIAG_PIN                         PF4   // M6-STOP, MIN6
#define E2_DIAG_PIN                         PF10  // M7-STOP, MIN7
#define E3_DIAG_PIN                         PC0   // M8-STOP, MIN8

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-STOP
  #else
    #define X_MIN_PIN                E0_DIAG_PIN  // M4-STOP
  #endif
#elif NEEDS_X_MINMAX
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-STOP
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-STOP
  #else
    #define Y_MIN_PIN                E1_DIAG_PIN  // M5-STOP
  #endif
#elif NEEDS_Y_MINMAX
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-STOP
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // M7-STOP
  #else
    #define Z_MIN_PIN                E2_DIAG_PIN  // M7-STOP
  #endif
#elif NEEDS_Z_MINMAX
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // M7-STOP
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // Z-STOP
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PG1
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PF3   // M5-STOP
#define FIL_RUNOUT2_PIN                     PF4   // M6-STOP

//
// Steppers
//
#define X_STEP_PIN                          PC14  // S1
#define X_DIR_PIN                           PC13
#define X_ENABLE_PIN                        PE6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD6
#endif

#define Y_STEP_PIN                          PE5   // S2
#define Y_DIR_PIN                           PE4
#define Y_ENABLE_PIN                        PE3
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD5
#endif

#define Z_STEP_PIN                          PE2   // S3
#define Z_DIR_PIN                           PE1
#define Z_ENABLE_PIN                        PE0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif

#define Z2_STEP_PIN                         PB9   // S4
#define Z2_DIR_PIN                          PB8
#define Z2_ENABLE_PIN                       PB7
#ifndef Z2_CS_PIN
  #define Z2_CS_PIN                         PD3
#endif

#define E0_STEP_PIN                         PG9   // S5
#define E0_DIR_PIN                          PG10
#define E0_ENABLE_PIN                       PG13
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD2
#endif

#define E1_STEP_PIN                         PG11  // S6
#define E1_DIR_PIN                          PD7
#define E1_ENABLE_PIN                       PG12
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PA15
#endif

#define E2_STEP_PIN                         PB4   // S7
#define E2_DIR_PIN                          PB3
#define E2_ENABLE_PIN                       PB5
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PA9
#endif

#define E3_STEP_PIN                         PG15  // S8
#define E3_DIR_PIN                          PB6
#define E3_ENABLE_PIN                       PG14
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PA10
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PB0   // TB
#if TEMP_SENSOR_0 == -5
  #define TEMP_0_CS_PIN                     PC9   // Max31865-0-CS
  #define TEMP_0_SCK_PIN                    PC10
  #define TEMP_0_MISO_PIN                   PC11
  #define TEMP_0_MOSI_PIN                   PC12
  #define SOFTWARE_SPI                            // Max31865 and LCD SD share a set of SPIs, Set SD to softwareSPI for Max31865
  #define FORCE_SOFT_SPI
#else
  #define TEMP_0_PIN                        PB1   // TH0
#endif
#if TEMP_SENSOR_1 == -5
  #define TEMP_1_CS_PIN                     PA8   // Max31865-1-CS
  #define TEMP_1_SCK_PIN                    TEMP_0_SCK_PIN
  #define TEMP_1_MISO_PIN                   TEMP_0_MISO_PIN
  #define TEMP_1_MOSI_PIN                   TEMP_0_MOSI_PIN
#else
  #define TEMP_1_PIN                        PC5   // TH1
#endif
#define TEMP_2_PIN                          PC4   // TH2
#define TEMP_3_PIN                          PA7   // TH3

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PF5   // Hotbed
#define HEATER_0_PIN                        PF6   // Heater0
#define HEATER_1_PIN                        PF7   // Heater1
#define HEATER_2_PIN                        PF9   // Heater2
#define HEATER_3_PIN                        PF8   // Heater3

#define FAN0_PIN                            PA0   // Fan0
#define FAN1_PIN                            PA1   // Fan1
#define FAN2_PIN                            PA2   // Fan2
#define FAN3_PIN                            PA3   // Fan3
#define FAN4_PIN                            PA4   // Fan4
// #define FAN5_PIN                            PA5   // Fan5, same channel with fan0
// #define FAN6_PIN                            PA6   // 4 wire Fan6
// #define FAN7_PIN                            PE8   // 4 wire Fan7

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PD10  // PS-ON
#endif

//
// Misc. Functions
//
#define LED_PIN                             PA14
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                      PC2
#endif
#ifndef FILWIDTH2_PIN
  #define FILWIDTH2_PIN                     PC3
#endif

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #if HAS_WIRED_LCD
    #define SDCARD_CONNECTION                LCD
  #else
    #define SDCARD_CONNECTION            ONBOARD
  #endif
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PC8
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PC7
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PC6
#endif

/**               ------                                      ------
 * (BEEPER) PG5  | 1  2 | PG4  (BTN_ENC)         (MISO) PE13 | 1  2 | PE12 (SCK)
 * (LCD_EN) PG3  | 3  4 | PG2  (LCD_RS)       (BTN_EN1) PG8  | 3  4 | PE11 (SD_SS)
 * (LCD_D4) PD15   5  6 | PD14 (LCD_D5)       (BTN_EN2) PG7    5  6 | PE14 (MOSI)
 * (LCD_D6) PD13 | 7  8 | PD12 (LCD_D7)     (SD_DETECT) PG6  | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PG5
#define EXP1_02_PIN                         PG4
#define EXP1_03_PIN                         PG3
#define EXP1_04_PIN                         PG2
#define EXP1_05_PIN                         PD15
#define EXP1_06_PIN                         PD14
#define EXP1_07_PIN                         PD13
#define EXP1_08_PIN                         PD12

#define EXP2_01_PIN                         PE13
#define EXP2_02_PIN                         PE12
#define EXP2_03_PIN                         PG8
#define EXP2_04_PIN                         PE11
#define EXP2_05_PIN                         PG7
#define EXP2_06_PIN                         PE14
#define EXP2_07_PIN                         PG6
#define EXP2_08_PIN                         -1


//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #ifndef SD_DETECT_STATE
    #define SD_DETECT_STATE HIGH
  #elif SD_DETECT_STATE == LOW
    #error "BOARD_BTT_KRAKEN_V1_0 onboard SD requires SD_DETECT_STATE set to HIGH."
  #endif
  #define SDSS                              PB12
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                        PB13
  #define SD_MISO_PIN                       PB14
  #define SD_MOSI_PIN                       PB15
  #define SD_DETECT_PIN                     PE15
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(LCD)
  #define SDSS                       EXP2_04_PIN
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
#endif

#if ENABLED(BTT_MOTOR_EXPANSION)
  /**
   *         ------                  ------
   * M3DIAG | 1  2 | M3RX     M3STP | 1  2 | M3DIR
   * M2DIAG | 3  4 | M2RX     M2STP | 3  4 | M2DIR
   * M1DIAG   5  6 | M1RX     M1DIR   5  6 | M1STP
   *   M3EN | 7  8 | M2EN      M1EN | 7  8 | --
   *    GND | 9 10 | --         GND | 9 10 | --
   *        ------                   ------
   *         EXP1                     EXP2
   */

  // M1 on Driver Expansion Module
  #define E4_STEP_PIN                EXP2_06_PIN
  #define E4_DIR_PIN                 EXP2_05_PIN
  #define E4_ENABLE_PIN              EXP2_07_PIN
  #define E4_DIAG_PIN                EXP1_05_PIN
  #define E4_CS_PIN                  EXP1_06_PIN
  #if HAS_TMC_UART
    #define E4_SERIAL_TX_PIN         EXP1_06_PIN
    #define E4_SERIAL_RX_PIN    E4_SERIAL_TX_PIN
  #endif

  // M2 on Driver Expansion Module
  #define E5_STEP_PIN                EXP2_03_PIN
  #define E5_DIR_PIN                 EXP2_04_PIN
  #define E5_ENABLE_PIN              EXP1_08_PIN
  #define E5_DIAG_PIN                EXP1_03_PIN
  #define E5_CS_PIN                  EXP1_04_PIN
  #if HAS_TMC_UART
    #define E5_SERIAL_TX_PIN         EXP1_04_PIN
    #define E5_SERIAL_RX_PIN    E5_SERIAL_TX_PIN
  #endif

  // M3 on Driver Expansion Module
  #define E6_STEP_PIN                EXP2_01_PIN
  #define E6_DIR_PIN                 EXP2_02_PIN
  #define E6_ENABLE_PIN              EXP1_07_PIN
  #define E6_DIAG_PIN                EXP1_01_PIN
  #define E6_CS_PIN                  EXP1_02_PIN
  #if HAS_TMC_UART
    #define E6_SERIAL_TX_PIN         EXP1_02_PIN
    #define E6_SERIAL_RX_PIN    E6_SERIAL_TX_PIN
  #endif

#endif // BTT_MOTOR_EXPANSION

//
// LCDs and Controllers
//
#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  /**
   *        ------                 ------            ---
   *       | 1  2 |               | 1  2 |            1 |
   *       | 3  4 |            RX | 3  4 | TX       | 2 | RX
   *   ENT   5  6 | BEEP      ENT   5  6 | BEEP     | 3 | TX
   *     B | 7  8 | A           B | 7  8 | A        | 4 |
   *   GND | 9 10 | VCC       GND | 9 10 | VCC        5 |
   *        ------                 ------            ---
   *         EXP1                   DWIN             TFT
   *
   * DWIN pins are labeled as printed on DWIN PCB. GND, VCC, A, B, ENT & BEEP can be connected in the same
   * orientation as the existing plug/DWIN to EXP1. TX/RX need to be connected to the TFT port, with TX->RX, RX->TX.
   */

  #ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
    #error "CAUTION! Ender-3 V2 display requires a custom cable. See 'pins_BTT_OCTOPUS_V1_common.h' for details. (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
  #endif

  #define BEEPER_PIN                 EXP1_06_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN
  #define BTN_ENC                    EXP1_05_PIN

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      //#define LCD_BACKLIGHT_PIN           -1
      #define LCD_RESET_PIN          EXP1_05_PIN  // Must be high or open for LCD to operate normally.
      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_06_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_07_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_08_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_06_PIN
      #endif
    #endif // !FYSETC_MINI_12864

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif
#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               120
  #define BOARD_ST7920_DELAY_2                80
  #define BOARD_ST7920_DELAY_3               580
#endif

#if HAS_SPI_TFT
  #define TFT_CS_PIN                 EXP2_04_PIN
  #define TFT_A0_PIN                 EXP2_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define TOUCH_INT_PIN              EXP1_07_PIN
  #define TOUCH_MISO_PIN             EXP1_06_PIN
  #define TOUCH_MOSI_PIN             EXP1_03_PIN
  #define TOUCH_SCK_PIN              EXP1_05_PIN
  #define TOUCH_CS_PIN               EXP1_04_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN
#endif

//
// NeoPixel LED
//
#ifndef NEOPIXEL_PIN
  #define NEOPIXEL_PIN                      PF12
#endif
#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PF11
#endif
