#include "gpio.h"
#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
// -----------------------------------------------------------------------------
// Fonction d'initialisation simple pour les GPIO utilisés
// -----------------------------------------------------------------------------
void initGPIO() {
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
    analogWrite(GPIO_PWM_HEATER, 0 ) ;  
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