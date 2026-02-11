#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cSensorsTask.h"
#include "RTOSQueues.h"
#include "globals.h"
#include "gpio.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();
BH1750 lightMeter;

static TaskHandle_t hSensors = nullptr;
static uint32_t readCount = 0;
static uint32_t errorCount = 0;

static void scanI2C() {
    byte error, address;
    int nDevices = 0;
    
    Serial.println("Scanning I2C bus...");
    
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("I2C device found at 0x%02X", address);
            
            if (address == 0x23) Serial.print(" (BH1750)");
            if (address == 0x44) Serial.print(" (SHT31)");
            if (address == 0x3C) Serial.print(" (OLED)");
            
            Serial.println();
            nDevices++;
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found");
    } else {
        Serial.printf("Found %d I2C device(s)\n", nDevices);
    }
}

bool checkSensorHealth() {
    float temp = sht31.readTemperature();
    float hum = sht31.readHumidity();
    bool sht31_ok = (!isnan(temp) && !isnan(hum));
    
    float lux = lightMeter.readLightLevel();
    bool bh1750_ok = (lux >= 0 && lux < 65535);
    
    if (!sht31_ok) {
        Serial.println("SHT31 health check failed - reinitializing");
        sht31.begin(SHT31_I2C_ADDR);
    }
    
    if (!bh1750_ok) {
        Serial.println("BH1750 health check failed - reinitializing");
        lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR);
    }
    
    return (sht31_ok && bh1750_ok);
}

static void sensorsTask(void *pv) {
    Serial.println("Rtos::Task Sensors > started.");

    uint32_t seq = 0;
    
    vTaskDelay(pdMS_TO_TICKS(500));
    checkSensorHealth();

    while (1) {
        digitalWrite(GPIO_BOARD_LED, HIGH);
        
        SensorData_t d{};
        
        d.temperature = sht31.readTemperature();
        d.humidity = sht31.readHumidity();
        d.lux = lightMeter.readLightLevel();
        d.ts_ms = millis();
        
        bool dataValid = (!isnan(d.temperature) && 
                         !isnan(d.humidity) && 
                         d.lux >= 0);
        
        if (dataValid) {
            sensorsModelWrite(d);
            readCount++;
            
            if (readCount % 10 == 0) {
                Serial.printf("Sample #%lu: TimeStamp=%lu, Temp=%.2f C, Hum=%.2f %%, Light=%.0f lux\n",
                             readCount, d.ts_ms, d.temperature, d.humidity, d.lux);
            }
            
        } else {
            errorCount++;
            Serial.printf("Sensor read error #%lu\n", errorCount);
            
            if (errorCount % 10 == 0) {
                checkSensorHealth();
            }
            
            d.temperature = -999.0f;
            d.humidity = -999.0f;
            d.lux = -1.0f;
            sensorsModelWrite(d);
        }
        
        digitalWrite(GPIO_BOARD_LED, LOW);
        
        seq++;
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

void sensorsInit() {
    Serial.println("Initializing I2C sensors");
    
    Wire.begin(GPIO_I2C_1_SDA, GPIO_I2C_1_SCL);
    Wire.setClock(100000);
    
    Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d @ 100kHz\n", 
                 GPIO_I2C_1_SDA, GPIO_I2C_1_SCL);
    
    //delay(100);                                       // Delay should not be used in freeRTOS, instead use vtaskdelay()
    vTaskDelay(pdMS_TO_TICKS(100));
    scanI2C();
    
    Serial.print("Initializing SHT31... ");
    if (!sht31.begin(SHT31_I2C_ADDR)) {
        Serial.println("FAILED");
        Serial.println("Check SHT31 wiring");
        while (1) {
            digitalWrite(GPIO_LED_RED, HIGH);
            //delay(200);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(GPIO_LED_RED, LOW);
            //delay(200);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    Serial.println("OK");
    
    Serial.print("Initializing BH1750... ");
    if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_I2C_ADDR)) {
        Serial.println("FAILED");
        Serial.println("Check BH1750 wiring");
        while (1) {
            digitalWrite(GPIO_LED_RED, HIGH);
            //delay(100);
            vTaskDelay(pdMS_TO_TICKS(100));
            digitalWrite(GPIO_LED_RED, LOW);
            //delay(100);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Our Code is Fail Operational and not Fail safe and monitoring has to be continued despite of any failure of initialization of components
    // The code above stucks in a while loop if failed to initialize the sensors

    Serial.println("OK");
    
    Serial.println("I2C sensors initialized successfully");
}

void sensorsStartTask() {
    xTaskCreate(sensorsTask, "Sensors", 4096, nullptr, 2, &hSensors);
    
    if (hSensors != nullptr) {
        Serial.println("Sensors task created");
    } else {
        Serial.println("Failed to create sensors task");
    }

}