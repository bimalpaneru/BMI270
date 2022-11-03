#include "bmi270.h"

#include "bmi270_config_file.h"

BMI270::BMI270(TwoWire *i2c_imu) { bmi_i2c = i2c_imu; }

BMI270::~BMI270() {}

/**
 * @brief Sets the communication mode type
 *
 * @param comms_mode: 0 for I3C 1 for SPI
 * @param address_or_cs_pin: default device address(0x68 for I2C depending on the state of SDO pin) or the cs_pin number
 * @return true if the chip ID is read properly i.e 0x24
 * @return false otherwise
 */
bool BMI270::begin(uint8_t comms_mode, uint8_t address_or_cs_pin) {
    this->_comms_mode = comms_mode;
    this->_address_or_cs_pin = address_or_cs_pin;

    return this->check_communications();
}

/**
 * @brief
 *
 * @param output: Address for read data
 * @param address: Device register address to be read
 * @param length: Number of bytes to read
 * @return true : if success
 * @return false : otherwise
 */
bool BMI270::read(uint8_t *output, bmi270_register_t address, uint16_t length) {
    bool success = false;

    success = bmi270_i2c_read(output, address, length);
    return success;
}

bool BMI270::read(bmi270_accelerometer_data &container) {
    uint8_t data[6];
    bool read_successful = read(data, BMI270_REGISTERS::ACCELEROMETER_DATA_START, 6);

    container.x = data[0] | int16_t(data[1]) << 8;
    container.y = data[2] | int16_t(data[3]) << 8;
    container.z = data[4] | int16_t(data[5]) << 8;

    return read_successful;
}

bool BMI270::bmi270_i2c_write(uint8_t *input, bmi270_register_t address, uint16_t length) {
    bool result = true;
    bmi_i2c->beginTransmission(_address_or_cs_pin);
    bmi_i2c->write(address);

    for (size_t i = 0; i < length; i++) {
        bmi_i2c->write(input[i]);
    }

    if (bmi_i2c->endTransmission() != 0) {
        result = false;
    }
    return result;
}

bool BMI270::bmi270_i2c_read(uint8_t *output, bmi270_register_t address, uint16_t length) {
    bool result = true;
    bmi_i2c->beginTransmission(_address_or_cs_pin);
    bmi_i2c->write(address);
    if (bmi_i2c->endTransmission() != 0) {
        result = false;
    }

    else  // OK, all worked, keep going
    {
        bmi_i2c->requestFrom(_address_or_cs_pin, length);
        for (size_t i = 0; (i < length) and Wire.available(); i++) {
            uint8_t c = Wire.read();
            output[i] = c;
        }
    }
    return result;
}

bool BMI270::write(uint8_t *input, bmi270_register_t address, uint16_t length) {
    bool success = false;
    success = bmi270_i2c_write(input, address, length);
    return success;
}

/**
 * @brief Runs the initialisation sequence for BMI270
 *
 * @return true if successful
 * @return false otherwise
 */
bool BMI270::sensor_init() {
    bool success = false;

    uint8_t pwr_conf_val = 0x00;
    uint8_t init_ctrl_val_1 = 0x00;
    uint8_t init_ctrl_val_2 = 0x01;
    uint8_t temp_value = 0x00;

    delayMicroseconds(600);
    bmi270_i2c_write(&pwr_conf_val, PWR_CONF, 1);
    delayMicroseconds(600);
    bmi270_i2c_write(&init_ctrl_val_1, INIT_CTRL, 1);
    i2c_write_init_data(bmi270_config_file);
    bmi270_i2c_write(&init_ctrl_val_2, INIT_CTRL, 1);

    bmi270_i2c_read(&temp_value, INTERNAL_STATUS, 1);

    if (temp_value & 0x01) {
        success = true;
    }
    return success;
}
/**
 * @brief Sets the device to normal  mode
 *
 */
void BMI270::normal_mode() {
    uint8_t pwr_ctrl_val = 0b00000110;
    uint8_t acc_conf_val = 0x0C;
    uint8_t gyro_conf_val = 0xA9;
    uint8_t pwr_conf_val = 0x02;

    bmi270_i2c_write(&pwr_ctrl_val, PWR_CTRL, 1);
    delay(1);
    bmi270_i2c_write(&acc_conf_val, ACC_CONF, 1);
    delay(1);
    bmi270_i2c_write(&gyro_conf_val, GYR_CONF, 1);
    delay(1);
    bmi270_i2c_write(&pwr_conf_val, PWR_CONF, 1);
}

/**
 * @brief Soft reset same as Power on Reset
 * Needs the Initi sequence after this reset to make the device functional
 *
 */
void BMI270::soft_reset() {
    uint8_t soft_reset_val = 0xB6;
    bmi270_i2c_write(&soft_reset_val, CMD, 1);
    delay(1000);
}

/**
 * @brief  Simple write to i2c device fuction with address to be written into with the data
 *
 * @param address: The register address where data is to be written
 * @param value : The data to be written to the address
 */
void BMI270::write(uint8_t address, uint8_t value) {
    Wire.beginTransmission(_address_or_cs_pin);
    Wire.write(address);
    Wire.write(value);
    Wire.endTransmission();
}

/**
 * @brief Writes the Initialisation sequence config's 8kB of data to the INIT register
 *
 * @param input Pointer to the array storing configuration data which is in bmi270_config_file.h
 */
void BMI270::i2c_write_init_data(const uint8_t *input) {
    uint8_t init_addr_0 = 0;
    uint8_t init_addr_1 = 0;

    for (uint16_t i = 0; i < 4096; i++) {
        init_addr_0 = (i & 0x0F);
        init_addr_1 = (i >> 4) & 0xFF;

        //-----------------Writing the base register of INIT_ADDR_0-----------------------//

        {
            Wire.beginTransmission(_address_or_cs_pin);
            Wire.write(INIT_ADDR_0);
            Wire.write(init_addr_0);
            Wire.endTransmission();
        }

        //-----------------Writing the base register of INIT_ADDR_1-----------------------//
        {
            Wire.beginTransmission(_address_or_cs_pin);
            Wire.write(INIT_ADDR_1);
            Wire.write(init_addr_1);
            Wire.endTransmission();
        }
        //------------------------------------------------------------------------//

        //----------Writing to the INIT DATA REGISTER---------------//
        {
            Wire.beginTransmission(_address_or_cs_pin);
            Wire.write(INIT_DATA);

            Wire.write(input[0]);
            Wire.write(input[1]);

            Wire.endTransmission();
        }
        //--------------------------------------------------------------------------------//

        input = input + 2;
    }
}

/**
 * @brief Not implemented yet
 *
 * @param addr
 * @return uint8_t
 */
uint8_t BMI270::get_config_file_data(uint16_t addr) { return bmi270_config_file[addr]; }

bool BMI270::check_communications() {
    bool success = false;
    uint8_t chip_id = 0x00;

    bmi270_i2c_read(&chip_id, CHIP_ID, 1);

    if (chip_id & DEFAULT_CHIP_ID) {
        success = true;
    }
    return success;
}