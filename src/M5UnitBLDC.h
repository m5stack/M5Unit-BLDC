/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __UNIT_BLDC_H
#define __UNIT_BLDC_H

#include "Arduino.h"
#include "Wire.h"

#define UNIT_BLDC_ADDR                     0x65
#define UNIT_BLDC_MODE_REG                 0x00
#define UNIT_BLDC_PWM_REG                  0x10
#define UNIT_BLDC_READBACK_RPM_REG         0x20
#define UNIT_BLDC_READBACK_FREQ_REG        0x30
#define UNIT_BLDC_SET_RPM_REG              0x40
#define UNIT_BLDC_PID_REG                  0x50
#define UNIT_BLDC_DIR_REG                  0x60
#define UNIT_BLDC_MOTOR_CONFIG_REG         0x70
#define UNIT_BLDC_MOTOR_STATUS_REG         0x80
#define UNIT_BLDC_READBACK_RPM_INT_REG     0x90
#define UNIT_BLDC_READBACK_FREQ_INT_REG    0xA0
#define UNIT_BLDC_READBACK_RPM_STRING_REG  0xB0
#define UNIT_BLDC_READBACK_FREQ_STRING_REG 0xC0
#define UNIT_BLDC_SET_RPM_INT_REG          0xD0
#define UNIT_BLDC_SET_LOWEST_PWM_REG       0xE0
#define SAVE_DATA_TO_FLASH_REG             0xF0
#define JUMP_TO_BOOTLOADER_REG             0xFD
#define FIRMWARE_VERSION_REG               0xFE
#define I2C_ADDRESS_REG                    0xFF

typedef enum { OPEN_LOOP_CTL_MODE = 0, CLOSED_LOOP_CTL_MODE } unit_bldc_control_mode_t;

typedef enum { MOTOR_FORWARD = 0, MOTOR_BACKWARD } unit_bldc_control_direction_t;

typedef enum {
    MOTOR_STANDBY = 0,
    MOTOR_RUNNING,
    MOTOR_ERROR,
} unit_bldc_motor_status_t;

class M5UnitBLDC {
   private:
    uint8_t _addr;
    TwoWire *_wire;
    uint8_t _scl;
    uint8_t _sda;
    uint8_t _speed;
    bool writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
    bool readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
    float bytes_to_float(uint8_t *s);
    void float_to_bytes(float s, uint8_t *d);

   public:
    /**
     * @brief
     *
     * @param wire I2C Port
     * @param addr addr device i2c addr
     * @param sda sda sda pin number
     * @param scl scl scl pin number
     * @param speed i2c speed
     * @return bool true=success / false=fail
     */
    bool begin(TwoWire *wire = &Wire, uint8_t addr = UNIT_BLDC_ADDR, uint8_t sda = 25, uint8_t scl = 21,
               uint32_t speed = 8000000L);

    /**
     * @brief Set the Mode object
     *
     * @param mode  0=open loop / 1=closed loop
     * @return bool true=success / false=fail
     */
    bool setMode(unit_bldc_control_mode_t mode);

    /**
     * @brief Get the Mode object
     *
     * @return uint8_t 0=open loop / 1=closed loop
     */
    uint8_t getMode(void);

    /**
     * @brief Set the Direction object
     *
     * @param dir  0=forward / 1=backward
     * @return bool true=success / false=fail
     */
    bool setDirection(unit_bldc_control_direction_t dir);

    /**
     * @brief Get the Direction object
     *
     * @return uint8_t 0=forward / 1=backward
     */
    uint8_t getDirection(void);

    /**
     * @brief Set PWM duty
     *
     * @param duty range: 0-2047
     * @return bool true=success / false=fail
     */
    bool setPWM(uint16_t duty);

    /**
     * @brief Get PWM duty
     *
     * @return uint16_t range: 0-2047
     */
    uint16_t getPWM(void);

    /**
     * @brief Set RPM
     *
     * @param rpm
     * @return bool
     */
    bool setRPM(float rpm);

    /**
     * @brief Get RPM
     *
     * @return float
     */
    float getRPM(void);

    /**
     * @brief Get the Rpm Real Time Readback
     *
     * @return float RPM value
     */
    float getRpmReadback(void);

    /**
     * @brief Get the Freq Real Time Readback
     *
     * @return float freq value
     */
    float getFreqReadback(void);

    /**
     * @brief Get the Rpm Readback String
     *
     * @return String RPM string
     */
    String getRpmReadbackString(void);

    /**
     * @brief Get the Freq Readback String
     *
     * @return String float freq string
     */
    String getFreqReadbackString(void);

    /**
     * @brief Set Closed Loop Mode PID parameter
     *
     * @param p Kp
     * @param i Ki
     * @param d Kd
     * @return bool true=success / false=fail
     */
    bool setPID(float p, float i, float d);

    /**
     * @brief Get Closed Loop Mode PID parameter
     *
     * @param p Kp pointer for get value
     * @param i Ki pointer for get value
     * @param d Kd pointer for get value
     * @return bool true=success / false=fail
     */
    bool getPID(float *p, float *i, float *d);

    /**
     * @brief Get the Motor Status object
     *
     * @return uint8_t 0=standby/1=running/2=error
     */
    uint8_t getMotorStatus(void);

    /**
     * @brief Set the Motor Model object
     *
     * @param model 0=Low Speed/1=High Speed
     * @return bool true=success / false=fail
     */
    bool setMotorModel(uint8_t model);

    /**
     * @brief Get the Motor Model object
     *
     * @return uint8_t 0=Low Speed/1=High Speed
     */
    uint8_t getMotorModel(void);

    /**
     * @brief Set the Motor Pole Pairs object
     *
     * @param pairs Motor Pole Pairs
     * @return bool true=success / false=fail
     */
    bool setMotorPolePairs(uint8_t pairs);

    /**
     * @brief Get the Motor Pole Pairs object
     *
     * @return uint8_t Motor Pole Pairs
     */
    uint8_t getMotorPolePairs(void);

    /**
     * @brief Save Motor Pole Pairs and Motor Mode Cfg to Internel flash
     *
     * @return bool true=success / false=fail
     */
    bool saveMotorDataToFlash(void);

    /**
     * @brief Get the Firmware Version object
     *
     * @return uint8_t version number
     */
    uint8_t getFirmwareVersion(void);

    /**
     * @brief Jump to Bootloader Mode, For Firmware Update Situation
     *
     * @return bool true=success / false=fail
     */
    bool jumpBootloader(void);

    /**
     * @brief Modify Device I2C Address
     *
     * @param addr new address
     * @return bool true=success / false=fail
     */
    bool setI2CAddress(uint8_t addr);
};

#endif
