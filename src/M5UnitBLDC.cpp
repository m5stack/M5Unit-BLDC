#include "M5UnitBLDC.h"

bool M5UnitBLDC::writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer,
                            uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    for (int i = 0; i < length; i++) {
        _wire->write(*(buffer + i));
    }
    uint8_t error = _wire->endTransmission();
    if (error == 0) {
        return true;
    } else {
        return false;
    }
}

bool M5UnitBLDC::readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer,
                           uint8_t length) {
    uint8_t index = 0;
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    if (_wire->requestFrom(addr, length)) {
        for (int i = 0; i < length; i++) {
            buffer[index++] = _wire->read();
        }
        return true;
    }
    return false;
}

void M5UnitBLDC::float_to_bytes(float s, uint8_t *d) {
    union {
        float value;
        uint8_t bytes[4];
    } f2b;
    f2b.value = s;
    memcpy(d, f2b.bytes, 4);
}

float M5UnitBLDC::bytes_to_float(uint8_t *s) {
    union {
        float value;
        uint8_t bytes[4];
    } f2b;
    memcpy(f2b.bytes, s, 4);
    return f2b.value;
}

bool M5UnitBLDC::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl,
                       uint32_t speed) {
    _wire  = wire;
    _addr  = addr;
    _sda   = sda;
    _scl   = scl;
    _speed = speed;
    _wire->begin(_sda, _scl);
    _wire->setClock(_speed);
    delay(10);
    _wire->beginTransmission(_addr);
    uint8_t error = _wire->endTransmission();
    if (error == 0) {
        return true;
    } else {
        return false;
    }
}

bool M5UnitBLDC::setMode(unit_bldc_control_mode_t mode) {
    uint8_t reg = UNIT_BLDC_MODE_REG;
    return writeBytes(_addr, reg, (uint8_t *)&mode, 1);
}

uint8_t M5UnitBLDC::getMode(void) {
    uint8_t mode;
    uint8_t reg = UNIT_BLDC_MODE_REG;
    readBytes(_addr, reg, (uint8_t *)&mode, 1);
    return mode;
}

bool M5UnitBLDC::setPWM(uint16_t duty) {
    uint8_t reg = UNIT_BLDC_PWM_REG;
    return writeBytes(_addr, reg, (uint8_t *)&duty, 2);
}

uint16_t M5UnitBLDC::getPWM(void) {
    uint16_t pwm_value;
    uint8_t reg = UNIT_BLDC_PWM_REG;
    readBytes(_addr, reg, (uint8_t *)&pwm_value, 2);
    return pwm_value;
}

bool M5UnitBLDC::setRPM(float rpm) {
    return writeBytes(_addr, UNIT_BLDC_SET_RPM_REG, (uint8_t *)&rpm, 4);
}

float M5UnitBLDC::getRPM(void) {
    uint8_t rx_data[4];
    float result = 0.0f;

    readBytes(_addr, UNIT_BLDC_SET_RPM_REG, (uint8_t *)&rx_data, 4);
    result = bytes_to_float(rx_data);

    return result;
}

float M5UnitBLDC::getRpmReadback(void) {
    uint8_t rx_data[4];
    float result = 0.0f;

    readBytes(_addr, UNIT_BLDC_READBACK_RPM_REG, (uint8_t *)&rx_data, 4);
    result = bytes_to_float(rx_data);

    return result;
}

String M5UnitBLDC::getRpmReadbackString(void) {
    char *p;
    uint8_t data[16];
    String res;

    readBytes(_addr, UNIT_BLDC_READBACK_RPM_STRING_REG, data, 16);
    p   = (char *)data;
    res = p;
    return res;
}

float M5UnitBLDC::getFreqReadback(void) {
    uint8_t rx_data[4];
    float result = 0.0f;

    readBytes(_addr, UNIT_BLDC_READBACK_FREQ_REG, (uint8_t *)&rx_data, 4);
    result = bytes_to_float(rx_data);

    return result;
}

String M5UnitBLDC::getFreqReadbackString(void) {
    char *p;
    uint8_t data[16];
    String res;

    readBytes(_addr, UNIT_BLDC_READBACK_FREQ_STRING_REG, data, 16);
    p   = (char *)data;
    res = p;
    return res;
}

bool M5UnitBLDC::setPID(float p, float i, float d) {
    uint8_t tx_data[12];

    int32_t p_int = p * 100;
    int32_t i_int = i * 100;
    int32_t d_int = d * 100;

    memcpy(tx_data, (uint8_t *)&p_int, 4);
    memcpy(tx_data + 4, (uint8_t *)&i_int, 4);
    memcpy(tx_data + 8, (uint8_t *)&d_int, 4);

    return writeBytes(_addr, UNIT_BLDC_PID_REG, (uint8_t *)&tx_data, 12);
}

bool M5UnitBLDC::getPID(float *p, float *i, float *d) {
    uint8_t rx_data[12];

    if (readBytes(_addr, UNIT_BLDC_PID_REG, (uint8_t *)&rx_data, 12)) {
        int32_t p_int;
        int32_t i_int;
        int32_t d_int;

        memcpy(&p_int, (uint8_t *)&rx_data[0], 4);
        memcpy(&i_int, (uint8_t *)&rx_data[4], 4);
        memcpy(&d_int, (uint8_t *)&rx_data[8], 4);

        *p = (float)p_int / 100.0;
        *i = (float)i_int / 100.0;
        *d = (float)d_int / 100.0;

        return true;
    }
    return false;
}

bool M5UnitBLDC::setDirection(unit_bldc_control_direction_t dir) {
    uint8_t tx_data[1];
    tx_data[0] = dir;
    return writeBytes(_addr, UNIT_BLDC_DIR_REG, tx_data, 1);
}

uint8_t M5UnitBLDC::getDirection(void) {
    uint8_t rx_data[4];

    readBytes(_addr, UNIT_BLDC_DIR_REG, (uint8_t *)&rx_data, 1);

    return rx_data[0];
}

uint8_t M5UnitBLDC::getMotorStatus(void) {
    uint8_t rx_data[4];

    readBytes(_addr, UNIT_BLDC_MOTOR_STATUS_REG, (uint8_t *)&rx_data, 1);

    return rx_data[0];
}

bool M5UnitBLDC::setMotorModel(uint8_t model) {
    return writeBytes(_addr, UNIT_BLDC_MOTOR_CONFIG_REG, (uint8_t *)&model, 1);
}

uint8_t M5UnitBLDC::getMotorModel(void) {
    uint8_t rx_data[4];

    readBytes(_addr, UNIT_BLDC_MOTOR_CONFIG_REG, (uint8_t *)&rx_data, 1);

    return rx_data[0];
}

bool M5UnitBLDC::setMotorPolePairs(uint8_t pairs) {
    return writeBytes(_addr, UNIT_BLDC_MOTOR_CONFIG_REG + 1, (uint8_t *)&pairs,
                      1);
}

uint8_t M5UnitBLDC::getMotorPolePairs(void) {
    uint8_t rx_data[4];

    readBytes(_addr, UNIT_BLDC_MOTOR_CONFIG_REG + 1, (uint8_t *)&rx_data, 1);

    return rx_data[0];
}

bool M5UnitBLDC::saveMotorDataToFlash(void) {
    uint8_t tx_data[1];
    tx_data[0] = 1;
    return writeBytes(_addr, SAVE_DATA_TO_FLASH_REG, tx_data, 1);
}

uint8_t M5UnitBLDC::getFirmwareVersion(void) {
    _wire->beginTransmission(_addr);
    _wire->write(FIRMWARE_VERSION_REG);
    _wire->endTransmission(false);

    uint8_t RegValue;

    _wire->requestFrom(_addr, 1);
    RegValue = Wire.read();
    return RegValue;
}

bool M5UnitBLDC::setI2CAddress(uint8_t addr) {
    uint8_t tx_data[1];
    tx_data[0] = addr;
    if (writeBytes(_addr, I2C_ADDRESS_REG, tx_data, 1)) {
        _addr = addr;
        return true;
    }
    return false;
}

bool M5UnitBLDC::jumpBootloader(void) {
    uint8_t value = 1;
    return writeBytes(_addr, JUMP_TO_BOOTLOADER_REG, (uint8_t *)&value, 1);
}
