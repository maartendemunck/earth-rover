// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__ADAFRUIT_BNO055_WRAPPER__
#define __EARTH_ROVER__ADAFRUIT_BNO055_WRAPPER__

#include "adafruit-bno055.hpp"
#include "eeprom-configuration-database-object.hpp"
#include <cstdint>

namespace earth_rover {

    struct EulerAngles_t {
        float yaw;
        float pitch;
        float roll;
    };

    template <typename I2CDevice_t>
    class AdafruitBNO055Wrapper : public EepromConfigDatabaseObject {

      private:
        I2CDevice_t &i2c_device;
        const uint8_t i2c_scl_pin;
        const uint8_t i2c_sda_pin;
        AdafruitBNO055<I2CDevice_t> bno055_device;
        bool bno055_device_is_configured;
        elapsedMillis time_since_last_calibration_save;

      public:
        AdafruitBNO055Wrapper(I2CDevice_t &i2c_device, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin)
            : i2c_device{i2c_device}, i2c_scl_pin{i2c_scl_pin}, i2c_sda_pin{i2c_sda_pin},
              bno055_device{i2c_device, decltype(bno055_device)::BNO055_ADDRESS_A},
              bno055_device_is_configured{false} {
            ;
        }

        void setup() {
            if(!bno055_device_is_configured) {
                i2c_device.begin(I2C_MASTER, 0x00, i2c_scl_pin, i2c_sda_pin, I2C_PULLUP_EXT, 400000,
                                 I2C_OP_MODE_ISR);
                bno055_device.setup();
                bno055_device_is_configured = true;
            }
        }

        void spinOnce() {
            ;
        }

        EulerAngles_t getOrientation() {
            EulerAngles_t angles;
            auto angles_raw = bno055_device.getEulerAngles();
            // Adjust Euler angles for the orientation of the Adafruit BNO055 IMU in the car.
            angles.yaw = -angles_raw.yaw - (M_PI / 2.);
            while(angles.yaw < 0) {
                angles.yaw += 2. * M_PI;
            }
            angles.pitch = angles_raw.pitch;
            angles.roll = -angles_raw.roll;
            return angles;
        }

        bool isFullyCalibrated() {
            auto calibration_status = bno055_device.getCalibrationStatus();
            return (calibration_status.system == 3 && calibration_status.accelerometer == 3
                    && calibration_status.gyroscope == 3 && calibration_status.magnetometer == 3);
        }

        bool isSaveRequested() override {
            return (EepromConfigDatabaseObject::isSaveRequested()
                    || time_since_last_calibration_save >= 10u * 60u * 1000u)
                   && (isFullyCalibrated() == true);
        }

        SerializationResult serialize(uint8_t *data, uint16_t size) {
            if(size >= 22u) {
                typename AdafruitBNO055<I2CDevice_t>::bno055_calibration_values_t
                    calibration_values;
                bno055_device.getCalibrationValues(calibration_values);
                data[0] = calibration_values.accel_offset_x & 0x00ff;
                data[1] = (calibration_values.accel_offset_x & 0xff00) >> 8;
                data[2] = calibration_values.accel_offset_y & 0x00ff;
                data[3] = (calibration_values.accel_offset_y & 0xff00) >> 8;
                data[4] = calibration_values.accel_offset_z & 0x00ff;
                data[5] = (calibration_values.accel_offset_z & 0xff00) >> 8;
                data[6] = calibration_values.mag_offset_x & 0x00ff;
                data[7] = (calibration_values.mag_offset_x & 0xff00) >> 8;
                data[8] = calibration_values.mag_offset_y & 0x00ff;
                data[9] = (calibration_values.mag_offset_y & 0xff00) >> 8;
                data[10] = calibration_values.mag_offset_z & 0x00ff;
                data[11] = (calibration_values.mag_offset_z & 0xff00) >> 8;
                data[12] = calibration_values.gyro_offset_x & 0x00ff;
                data[13] = (calibration_values.gyro_offset_x & 0xff00) >> 8;
                data[14] = calibration_values.gyro_offset_y & 0x00ff;
                data[15] = (calibration_values.gyro_offset_y & 0xff00) >> 8;
                data[16] = calibration_values.gyro_offset_z & 0x00ff;
                data[17] = (calibration_values.gyro_offset_z & 0xff00) >> 8;
                data[18] = calibration_values.accel_radius & 0x00ff;
                data[19] = (calibration_values.accel_radius & 0xff00) >> 8;
                data[20] = calibration_values.mag_radius & 0x00ff;
                data[21] = (calibration_values.mag_radius & 0xff00) >> 8;
                for(unsigned index = 22; index < size; ++index) {
                    data[index] = 0xffu;
                }
                time_since_last_calibration_save = 0u;
                return SerializationResult::SAVE_IN_NEW_RECORD;
            }
            else {
                return SerializationResult::ERROR;
            }
        }

        bool deserialize(uint8_t *data, uint16_t size) {
            if(size >= 22u) {
                typename decltype(bno055_device)::bno055_calibration_values_t calibration_values;
                calibration_values.accel_offset_x = (data[1] << 8) | data[0];
                calibration_values.accel_offset_y = (data[3] << 8) | data[2];
                calibration_values.accel_offset_z = (data[5] << 8) | data[4];
                calibration_values.mag_offset_x = (data[7] << 8) | data[6];
                calibration_values.mag_offset_y = (data[9] << 8) | data[8];
                calibration_values.mag_offset_z = (data[11] << 8) | data[10];
                calibration_values.gyro_offset_x = (data[13] << 8) | data[12];
                calibration_values.gyro_offset_y = (data[15] << 8) | data[14];
                calibration_values.gyro_offset_z = (data[17] << 8) | data[16];
                calibration_values.accel_radius = (data[19] << 8) | data[18];
                calibration_values.mag_radius = (data[21] << 8) | data[20];
                if(!bno055_device_is_configured) {
                    setup();
                }
                bno055_device.setCalibrationValues(calibration_values);
                time_since_last_calibration_save = 0u;
                return false;
            }
            else {
                return true;
            }
        }
    };

}  // namespace earth_rover

#endif