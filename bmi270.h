#ifndef BMI270_H
#define BMI270_H

#include <Arduino.h>
#include <ArduinoLog.h>
#include <SPI.h>
#include <Wire.h>

#include "bmi270_config_file.h"
/////////////////////////////////////////////////////////////////////////////
const uint8_t BMI270_DEFAULT_DEVICE_ADDRESS = 0x68;
const uint8_t DEFAULT_CHIP_ID = 0x24;
////////////////////////////////////////////////////////////////////////////
enum BMI270_COMMUNICATION_MODE {
    BMI270_I2C_MODE = 0,
    BMI270_SPI_MODE = 1,
};

typedef struct ACCELEROMETER_DATA {
    int16_t x;
    int16_t y;
    int16_t z;

} bmi270_accelerometer_data;

typedef struct GYROSCOPE_DATA {
    int16_t x;
    int16_t y;
    int16_t z;
} bmi270_gyroscope_data;

typedef struct CHIP_ID {
    uint8_t chip_id;
} bmi270_chip_id;

class BMI270 {
   public:
    BMI270(TwoWire *i2c_imu);
    ~BMI270();
    bool begin(uint8_t comms_mode = BMI270_I2C_MODE, uint8_t address_or_cs_pin = BMI270_DEFAULT_DEVICE_ADDRESS);  // or the defualt address is 0x68
    bool read(bmi270_chip_id &);
    bool read(bmi270_accelerometer_data &);
    bool read(bmi270_gyroscope_data &);
    bool sensor_init();

    typedef enum BMI270_REGISTERS {
        CMD = 0x7E,
        PWR_CTRL = 0x7D,
        PWR_CONF = 0x7C,
        OFFSET_6 = 0x77,             // gyr_gain_e
        OFFSET_5 = 0x76,             // gyr_usr_off_z_7_0
        OFFSET_4 = 0x75,             // gyr_usr_off_y_7_0
        OFFSET_3 = 0x74,             // gyr_usr_off_x_7_0
        OFFSET_2 = 0x73,             // off_acc_z
        OFFSET_1 = 0x72,             // off_acc_y
        OFFSET_0 = 0x71,             // off_acc_x
        NV_CONF = 0x70,              // reserved acc_off_en i2c_wdt_en i2c_wdt_sel spi_en
        GYR_SELF_TEST_A_XES = 0x6E,  // reserved gyr_axis_z_ok gyr_axis_y_ok
                                     // gyr_axis_x_ok gyr_st_axes_done
        ACC_SELF_TEST = 0x6D,
        DRV = 0x6C,
        IF_CONF = 0x6B,
        NVM_CON = 0x6A,
        GYR_CRT = 0x69,
        AUX_IF_T = 0x68,
        INTERNAL_ERROR = 0x5F,
        INIT_DATA = 0x5E,
        INIT_ADDR_1 = 0x5C,
        INIT_ADDR_0 = 0x5B,
        INIT_CTRL = 0x59,
        INT_MAP_DATA = 0x58,
        INT2_MAP_FEAT = 0x57,
        INT1_MAP_FEAT = 0x56,
        INT_LATCH = 0x55,
        INT2_IO_CTRL = 0x54,
        INT1_IO_CTRL = 0x53,
        AUX_WR_DATA = 0x4F,
        AUX_WR_ADDR = 0x4E,
        AUX_RD_ADDR = 0x4D,
        AUX_IF_CONF = 0x4C,
        AUX_DEV_ID = 0x4B,
        SATURATION = 0x4A,
        FIFO_CONFIG_1 = 0x49,
        FIFO_CONFIG_0 = 0x48,
        FIFO_WTM_1 = 0x47,
        FIFO_WTM_0 = 0x46,
        FIFO_DO_WNS = 0x45,
        AUX_CONFF = 0x44,
        GYR_RANGE = 0x43,
        GYR_CONF = 0x42,
        ACC_RAN = 0x41,
        ACC_CONF = 0x40,
        FEATURE = 0x3F,
        FIFO_DATA = 0x26,
        FIFO_LENGTH_1 = 0x25,
        FIFO_LENGTH_0 = 0x24,
        TEMPERATURE_1 = 0x23,
        TEMPERATURE_0 = 0x22,
        INTERNAL_STATUS = 0x21,
        WR_GEST = 0x20,
        SC_OUT_1 = 0x1F,
        SC_OUT_0 = 0x1E,
        INT_STATUS_1 = 0x1D,
        INT_STATUS_0 = 0x1C,
        EVENT = 0x1B,
        SENSORTIME_2 = 0x1A,
        SENSORTIME_1 = 0x19,
        SENSORTIME_0 = 0x18,
        DATA_19 = 0x17,
        DATA_18 = 0x16,
        DATA_17 = 0x15,
        DATA_16 = 0x14,
        DATA_15 = 0x13,  // gyr_x_15_8
        DATA_14 = 0x12,  // gyr_x_7_0
        GYROSCOPE_DATA_START = 0x12,
        DATA_13 = 0x11,  // acc_z_15_8
        DATA_12 = 0x10,  // acc_z_7_0
        DATA_11 = 0x0F,  // acc_y_15_8
        DATA_10 = 0x0E,  // acc_y_7_0
        DATA_9 = 0x0D,   // acc_x_15_8
        DATA_8 = 0x0C,   // acc_x_7_0
        ACCELEROMETER_DATA_START = 0x0C,
        DATA_7 = 0x0B,    // aux_r_15_8
        DATA_6 = 0x0A,    // aux_r_7_0
        DATA_5 = 0x09,    // aux_z_15_8
        DATA_4 = 0x08,    // aux_z_7_0
        DATA_3 = 0x07,    // aux_y_15_8
        DATA_2 = 0x06,    // aux_y_7_0
        DATA_1 = 0x05,    // aux_x_15_8
        DATA_0 = 0x04,    // aux_x_7_0
        STATUS = 0x03,    // data ready for accelerometer and gyro
        ERR_REG = 0x02,   // aux_err fifo_err reserved internal_err fatal_err
        reserved = 0x01,  //
        CHIP_ID = 0x00    //  chip_id
    } bmi270_register_t;

    void normal_mode();
    void soft_reset();
    void load_config_settings();
    void i2c_write_init_data(const uint8_t *input);

    //  private:
    uint8_t _comms_mode;
    uint8_t _address_or_cs_pin;
    uint8_t _device_type;
    TwoWire *bmi_i2c;

    void write(uint8_t address, uint8_t value);
    void read(uint8_t address, uint8_t value);

    bool write(uint8_t *input, bmi270_register_t address, uint16_t length = 1);
    bool read(uint8_t *output, bmi270_register_t address, uint16_t length = 1);

    bool bmi270_i2c_write(uint8_t *input, bmi270_register_t address, uint16_t length);
    bool bmi270_i2c_read(uint8_t *output, bmi270_register_t address, uint16_t length);

    uint8_t get_config_file_data(uint16_t addr);
    bool check_communications();
};

#endif