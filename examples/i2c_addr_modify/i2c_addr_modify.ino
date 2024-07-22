/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * @file i2c_addr_modify.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief
 * @version 0.1
 * @date 2024-04-11
 *
 *
 * @Hardwares: M5Core + Unit BLDC
 * @Platform Version: Arduino M5Stack Board Manager v2.1.1
 * @Dependent Library:
 * M5UnitBLDC: https://github.com/m5stack/M5Unit-BLDC
 */

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