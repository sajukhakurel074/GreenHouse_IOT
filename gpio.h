//#include "esp32-hal.h"
//#include "esp32-hal-gpio.h"
#pragma once

////////////////////////////////////////////////////////////////////////////////
// Prototype Connected GreenHouse Heltec V3 LoRa(32)  GPIO Pinout
////////////////////////////////////////////////////////////////////////////////

//------J2 Header right [RST] Button -- Top ----
#define GPIO_ENC_DT             19  // GPIO(19) J2:18  <== Encoder Data 
#define GPIO_ENC_CLK            20  // GPIO(20) J2:17  <== Encoder Clock  20/T  
//                                     GPIO(21) J2:16  ==> [OLED_RST]      
//                                     GPIO(20) J2:15  ==> [SPICS1] 
#define GPIO_I2C_1_SCL          48  // GPIO(48) J2:14  ==>  SCL [BH1750 Light] == [(Yellow)  SHT31 Temp./ Humidity] 
#define GPIO_I2C_1_SDA          47  // GPIO(47) J2:13  ==>  SDA [BH1750 Light] == [(Green) SHT31 Temp./ Humidity] 
#define GPIO_U1TXD              33  // GPIO(33) J2:12  ==> UART 
#define GPIO_U1RXD              34  // GPIO(34) J2:11  <== UART 
#define GPIO_BOARD_LED          35  // GPIO(35) J2:10  ==> S01 MOSFET-N 100V/14A 
#define GPIO_VEXT_PIN           36  // GPIO(36) J2:09  ==> 3.3 volts 
#define GPIO_BOARD_BP            0  // GPIO(00) J2:08  <== Encoder switch 
//                                              J2:07  ==> [RST]     
//                                              J2:06  ==> [USB Tx]    
//                                              J2:05  <== [USB Rx]     
//                                              J2:04  <== Ve 
//                                              J2:03  <== Ve       |==> Encoder Supply  
//                                              J2:02  <== Vin +5 ==|<== [DC-DC] <== 12 Volts.     
//                                              J2:01  GROUND     ==> Encoder  
//------J3 Header left [PRG] button -- Top ---
#define GPIO_PWM_HEATER          7  // GPIO(7)  J3:18  ==> S07 MOSFET-N 100V/14A 
#define GPIO_FAN                 6  // GPIO(6)  J3:17  ==> S06 MOSFET-N 100V/14A 
#define GPIO_FREE_5              5  // GPIO(5)  J3:16  ==> S05 MOSFET-N 100V/14A
#define GPIO_FREE_4              4  // GPIO(4)  J3:15  ==> S04 MOSFET-N 100V/14A
#define GPIO_FREE_3              3  // GPIO(3)  J3:14  ==> S03 MOSFET-N 100V/14A
#define GPIO_LED_RED             2  // GPIO(2)  J3:13  ==> S02 MOSFET-N 100V/14A
#define GPIO_VBAT_PIN            1  // GPIO(1)  J3:12  <== 

#define GPIO_SDA_PIN            45  // GPIO(45) J3:06  <==> I2C SDA LCD Display  
#define GPIO_SCL_PIN            46  // GPIO(46) J3:05  <==> I2C SCL Sensors 
//                                              J3:04    
//                                              J3:03  ==> 3.3 Volts ==>  
//                                              J3:02  ==> 3.3 Volts  ==> BH1750, SHT31    
//                                              J3:01  GROUND         ==> BH1750, SHT31  

// -----------------------------------------------------------------------------
// Fonction d'initialisation simple pour les GPIO utilisés
// -----------------------------------------------------------------------------
inline void initGPIO() {
    // LEDs
    pinMode(GPIO_BOARD_LED, OUTPUT);
    pinMode(GPIO_LED_RED, OUTPUT);

    // Encodeur
    pinMode(GPIO_ENC_DT, INPUT);
    pinMode(GPIO_ENC_CLK, INPUT);
    pinMode(GPIO_BOARD_BP, INPUT_PULLUP);

    // PWM / MOSFET (en sortie)
    pinMode(GPIO_PWM_HEATER, OUTPUT);
    pinMode(GPIO_FAN, OUTPUT);
    pinMode(GPIO_FREE_3, OUTPUT);
    pinMode(GPIO_FREE_4, OUTPUT);
    pinMode(GPIO_FREE_5, OUTPUT);

    //VEXT 
    pinMode(GPIO_VEXT_PIN, OUTPUT);
    digitalWrite (GPIO_VEXT_PIN, LOW); // ON 

    // Force OFF at startup 
    analogWrite(GPIO_PWM_HEATER, 155 ) ;  
    digitalWrite (GPIO_FAN , LOW ) ;  
    

    // I2C BH1750 et OLED seront initialisés par Wire / TwoWire
}

// -----------------------------------------------------------------------------
//   FS400-SHT31  PIN OUT 
// -----------------------------------------------------------------------------
/*
I2C (slave): 0x44 (hex)
🔴 Rouge  → VCC (3.3V  o 5V)
⚫ Noir   → GND
🟡 Jaune  → SDA 
🟢 Vert   → SCL 
*/ 




