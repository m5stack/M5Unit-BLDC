#include "M5UnitBLDC.h"

M5UnitBLDC driver;

void setup() {
    delay(2000);
    Serial.begin(115200);
    while (!driver.begin(&Wire, UNIT_BLDC_ADDR, 21, 22, 200000U)) {
        Serial.println("Couldn't find Unit BLDC");
        delay(2000);
    }

    if (driver.setI2CAddress(UNIT_BLDC_ADDR + 1)) {
        Serial.printf("Modify BLDC I2C Addr to  %02X", UNIT_BLDC_ADDR + 1);
    } else {
        Serial.println("I2C Addr Modify Fail");
    }
}

void loop() {
}