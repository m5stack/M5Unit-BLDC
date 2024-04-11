/**
 * @file pwm.ino
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
    Serial.begin(115200);

    while (!driver.begin(&Wire, UNIT_BLDC_ADDR, 21, 22, 200000U)) {
        Serial.println("Couldn't find Unit BLDC");
        delay(2000);
    }

    if (driver.setMode(OPEN_LOOP_CTL_MODE)) {
        Serial.println("set BLDC driver to Open Loop Control Mode");
    }
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

void loop() {
    // before change motor direction you need set the PWM/RPM to 0
    driver.setPWM(0);
    printMotorStatus();
    delay(2000);
    if (driver.setDirection(MOTOR_FORWARD)) {
        Serial.println("set BLDC Motor Forward");
    }
    // PWM Duty Range: 0-2047
    driver.setPWM(2000);
    printMotorStatus();
    delay(2000);

    // before change motor direction you need set the PWM/RPM to 0
    driver.setPWM(0);
    printMotorStatus();
    delay(2000);
    if (driver.setDirection(MOTOR_BACKWARD)) {
        Serial.println("set BLDC Motor Backward");
    }
    // PWM Duty Range: 0-2047
    driver.setPWM(2000);
    printMotorStatus();
    delay(2000);
}
