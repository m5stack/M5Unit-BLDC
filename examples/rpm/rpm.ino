/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * @file rpm.ino
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

#define MOTOR_POLE_PAIRS 7

void setup() {
    Serial.begin(115200);

    while (!driver.begin(&Wire, UNIT_BLDC_ADDR, 21, 22, 200000U)) {
        Serial.println("Couldn't find Unit BLDC");
        delay(2000);
    }

    // only closed loop mode support RPM control
    if (driver.setMode(CLOSED_LOOP_CTL_MODE)) {
        Serial.println("set BLDC driver to Closed Loop Control Mode");
    }

    if (driver.setMotorModel(1)) {
        Serial.println("set BLDC driver to High Speed Mode");
    }

    if (driver.setMotorPolePairs(MOTOR_POLE_PAIRS)) {
        Serial.printf("set MotorPolePairs: %d\r\n", MOTOR_POLE_PAIRS);
    }

    if (driver.saveMotorDataToFlash()) {
        Serial.println("commit Motor Data");
    }
}

void printMotorRPM() {
    Serial.printf("BLDC Motor RPM: %f\r\n", driver.getRpmReadback());
    Serial.printf("BLDC Motor Pulse Freq: %f\r\n", driver.getFreqReadback());
}

void printMotorStatus() {
    uint8_t status = driver.getMotorStatus();
    switch (status) {
        case MOTOR_STANDBY:
            Serial.println("BLDC Motor Standby");
            break;
        case MOTOR_RUNNING:
            Serial.println("BLDC Motor Running");
            break;
        case MOTOR_ERROR:
            Serial.println("BLDC Motor Error");
            break;
        default:
            break;
    }
}

unsigned long start_time = 0;

void loop() {
    // before change motor direction you need set the PWM/RPM to 0
    driver.setRPM(0);
    printMotorStatus();
    delay(2000);
    if (driver.setDirection(MOTOR_FORWARD)) {
        Serial.println("set BLDC Motor Forward");
    }
    driver.setRPM(700.0);
    printMotorStatus();
    start_time = millis();
    while (millis() - start_time < 10000) {
        printMotorRPM();
    }

    // before change motor direction you need set the PWM/RPM to 0
    driver.setRPM(0);
    printMotorStatus();
    delay(2000);
    if (driver.setDirection(MOTOR_BACKWARD)) {
        Serial.println("set BLDC Motor Backward");
    }
    driver.setRPM(700.0);
    printMotorStatus();
    start_time = millis();
    while (millis() - start_time < 10000) {
        printMotorRPM();
    }
}
