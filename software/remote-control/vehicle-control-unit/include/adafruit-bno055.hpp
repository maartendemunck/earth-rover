// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__ADAFRUIT_BNO055__
#define __EARTH_ROVER__ADAFRUIT_BNO055__

namespace earth_rover {

    template <typename I2CDevice> class AdafruitBNO055 {

      public:
        static constexpr uint8_t BNO055_ID{0xA0};
        static constexpr uint8_t BNO055_ADDRESS_A{0x28};
        static constexpr uint8_t BNO055_ADDRESS_B{0x29};

        struct bno055_calibration_status_t {
            uint8_t system;
            uint8_t accelerometer;
            uint8_t gyroscope;
            uint8_t magnetometer;
        };

        struct bno055_calibration_values_t {
            int16_t accel_offset_x;
            int16_t accel_offset_y;
            int16_t accel_offset_z;
            int16_t mag_offset_x;
            int16_t mag_offset_y;
            int16_t mag_offset_z;
            int16_t gyro_offset_x;
            int16_t gyro_offset_y;
            int16_t gyro_offset_z;
            int16_t accel_radius;
            int16_t mag_radius;
        };

        struct bno055_vector_t {
            float x;
            float y;
            float z;
        };

        struct bno055_quaternion_t {
            float w;
            float x;
            float y;
            float z;
        };

        struct bno055_euler_angles_t {
            float yaw;
            float pitch;
            float roll;
        };

      private:
        enum class bno055_register_t : uint8_t {
            BNO055_CHIP_ID_ADDR = 0x00,
            BNO055_ACCEL_REV_ID_ADDR = 0x01,
            BNO055_MAG_REV_ID_ADDR = 0x02,
            BNO055_GYRO_REV_ID_ADDR = 0x03,
            BNO055_SW_REV_ID_LSB_ADDR = 0x04,
            BNO055_SW_REV_ID_MSB_ADDR = 0x05,
            BNO055_BL_REV_ID_ADDR = 0x06,
            BNO055_PAGE_ID_ADDR = 0x07,
            BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08,
            BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09,
            BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A,
            BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0B,
            BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C,
            BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0D,
            BNO055_MAG_DATA_X_LSB_ADDR = 0x0E,
            BNO055_MAG_DATA_X_MSB_ADDR = 0x0F,
            BNO055_MAG_DATA_Y_LSB_ADDR = 0x10,
            BNO055_MAG_DATA_Y_MSB_ADDR = 0x11,
            BNO055_MAG_DATA_Z_LSB_ADDR = 0x12,
            BNO055_MAG_DATA_Z_MSB_ADDR = 0x13,
            BNO055_GYRO_DATA_X_LSB_ADDR = 0x14,
            BNO055_GYRO_DATA_X_MSB_ADDR = 0x15,
            BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16,
            BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17,
            BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18,
            BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19,
            BNO055_EULER_H_LSB_ADDR = 0x1A,
            BNO055_EULER_H_MSB_ADDR = 0x1B,
            BNO055_EULER_R_LSB_ADDR = 0x1C,
            BNO055_EULER_R_MSB_ADDR = 0x1D,
            BNO055_EULER_P_LSB_ADDR = 0x1E,
            BNO055_EULER_P_MSB_ADDR = 0x1F,
            BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20,
            BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21,
            BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22,
            BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23,
            BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24,
            BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25,
            BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26,
            BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27,
            BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28,
            BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29,
            BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2A,
            BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2B,
            BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2C,
            BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2D,
            BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2E,
            BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2F,
            BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30,
            BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31,
            BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32,
            BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33,
            BNO055_TEMP_ADDR = 0x34,
            BNO055_CALIB_STAT_ADDR = 0x35,
            BNO055_SELFTEST_RESULT_ADDR = 0x36,
            BNO055_INTR_STAT_ADDR = 0x37,
            BNO055_SYS_CLK_STAT_ADDR = 0x38,
            BNO055_SYS_STAT_ADDR = 0x39,
            BNO055_SYS_ERR_ADDR = 0x3A,
            BNO055_UNIT_SEL_ADDR = 0x3B,
            BNO055_DATA_SELECT_ADDR = 0x3C,
            BNO055_OPR_MODE_ADDR = 0x3D,
            BNO055_PWR_MODE_ADDR = 0x3E,
            BNO055_SYS_TRIGGER_ADDR = 0x3F,
            BNO055_TEMP_SOURCE_ADDR = 0x40,
            BNO055_AXIS_MAP_CONFIG_ADDR = 0x41,
            BNO055_AXIS_MAP_SIGN_ADDR = 0x42,
            BNO055_SIC_MATRIX_0_LSB_ADDR = 0x43,
            BNO055_SIC_MATRIX_0_MSB_ADDR = 0x44,
            BNO055_SIC_MATRIX_1_LSB_ADDR = 0x45,
            BNO055_SIC_MATRIX_1_MSB_ADDR = 0x46,
            BNO055_SIC_MATRIX_2_LSB_ADDR = 0x47,
            BNO055_SIC_MATRIX_2_MSB_ADDR = 0x48,
            BNO055_SIC_MATRIX_3_LSB_ADDR = 0x49,
            BNO055_SIC_MATRIX_3_MSB_ADDR = 0x4A,
            BNO055_SIC_MATRIX_4_LSB_ADDR = 0x4B,
            BNO055_SIC_MATRIX_4_MSB_ADDR = 0x4C,
            BNO055_SIC_MATRIX_5_LSB_ADDR = 0x4D,
            BNO055_SIC_MATRIX_5_MSB_ADDR = 0x4E,
            BNO055_SIC_MATRIX_6_LSB_ADDR = 0x4F,
            BNO055_SIC_MATRIX_6_MSB_ADDR = 0x50,
            BNO055_SIC_MATRIX_7_LSB_ADDR = 0x51,
            BNO055_SIC_MATRIX_7_MSB_ADDR = 0x52,
            BNO055_SIC_MATRIX_8_LSB_ADDR = 0x53,
            BNO055_SIC_MATRIX_8_MSB_ADDR = 0x54,
            ACCEL_OFFSET_X_LSB_ADDR = 0x55,
            ACCEL_OFFSET_X_MSB_ADDR = 0x56,
            ACCEL_OFFSET_Y_LSB_ADDR = 0x57,
            ACCEL_OFFSET_Y_MSB_ADDR = 0x58,
            ACCEL_OFFSET_Z_LSB_ADDR = 0x59,
            ACCEL_OFFSET_Z_MSB_ADDR = 0x5A,
            MAG_OFFSET_X_LSB_ADDR = 0x5B,
            MAG_OFFSET_X_MSB_ADDR = 0x5C,
            MAG_OFFSET_Y_LSB_ADDR = 0x5D,
            MAG_OFFSET_Y_MSB_ADDR = 0x5E,
            MAG_OFFSET_Z_LSB_ADDR = 0x5F,
            MAG_OFFSET_Z_MSB_ADDR = 0x60,
            GYRO_OFFSET_X_LSB_ADDR = 0x61,
            GYRO_OFFSET_X_MSB_ADDR = 0x62,
            GYRO_OFFSET_Y_LSB_ADDR = 0x63,
            GYRO_OFFSET_Y_MSB_ADDR = 0x64,
            GYRO_OFFSET_Z_LSB_ADDR = 0x65,
            GYRO_OFFSET_Z_MSB_ADDR = 0x66,
            ACCEL_RADIUS_LSB_ADDR = 0x67,
            ACCEL_RADIUS_MSB_ADDR = 0x68,
            MAG_RADIUS_LSB_ADDR = 0x69,
            MAG_RADIUS_MSB_ADDR = 0x6A
        };

        enum class bno055_operation_mode_t : uint8_t {
            OPERATION_MODE_CONFIG = 0x00,
            OPERATION_MODE_ACCONLY = 0x01,
            OPERATION_MODE_MAGONLY = 0x02,
            OPERATION_MODE_GYRONLY = 0x03,
            OPERATION_MODE_ACCMAG = 0x04,
            OPERATION_MODE_ACCGYRO = 0x05,
            OPERATION_MODE_MAGGYRO = 0x06,
            OPERATION_MODE_AMG = 0x07,
            OPERATION_MODE_IMUPLUS = 0x08,
            OPERATION_MODE_COMPASS = 0x09,
            OPERATION_MODE_M4G = 0x0A,
            OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
            OPERATION_MODE_NDOF = 0x0C
        };

        enum class bno055_power_mode_t : uint8_t {
            POWER_MODE_NORMAL = 0x00,
            POWER_MODE_LOWPOWER = 0x01,
            POWER_MODE_SUSPEND = 0x02
        };

        I2CDevice &i2c_device;
        uint8_t i2c_address;
        bno055_operation_mode_t bno055_operation_mode;

      public:
        AdafruitBNO055(I2CDevice &i2c_device, uint8_t i2c_address = BNO055_ADDRESS_A)
            : i2c_device{i2c_device}, i2c_address{i2c_address},
              bno055_operation_mode{bno055_operation_mode_t::OPERATION_MODE_CONFIG} {
            ;
        }

        bool setup() {
            // Wait for device to appear.
            bool device_available = false;
            int8_t max_tries = 20;
            while(!device_available && max_tries-- > 0) {
                if(readByte(bno055_register_t::BNO055_CHIP_ID_ADDR) == BNO055_ID) {
                    device_available = true;
                }
                else {
                    delay(50);
                }
            }
            if(!device_available) {
                return false;
            }
            // Switch device to config mode.
            setOperationMode(bno055_operation_mode_t::OPERATION_MODE_CONFIG);
            // Reset device.
            writeByte(bno055_register_t::BNO055_SYS_TRIGGER_ADDR, 0x20);
            while(delay(50), readByte(bno055_register_t::BNO055_CHIP_ID_ADDR) != BNO055_ID) {
                ;
            }
            // Set to normal power mode.
            setPowerMode(bno055_power_mode_t::POWER_MODE_NORMAL);
            delay(20);
            // Select page 0 of register map.
            writeByte(bno055_register_t::BNO055_PAGE_ID_ADDR, 0x00);
            // Set output units.
            uint8_t output_units = (0 << 7) |  // Orientation: Android
                                   (0 << 4) |  // Temperature: °C
                                   (1 << 2) |  // Euler angles: radians
                                   (1 << 1) |  // Gyroscope: radians
                                   (0 << 0);  // Accelerometer: m/s^2
            writeByte(bno055_register_t::BNO055_UNIT_SEL_ADDR, output_units);
            // Set system trigger register.
            writeByte(bno055_register_t::BNO055_SYS_TRIGGER_ADDR, 0x00);
            delay(10);
            // Set to NDOF mode.
            setOperationMode(bno055_operation_mode_t::OPERATION_MODE_NDOF);
            delay(20);
            // Setup succeeded.
            return false;
        }

        bno055_quaternion_t getQuaternion() {
            uint8_t buffer[8];
            memset(buffer, 0x00, 8);
            readNBytes(bno055_register_t::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
            // Translate raw values to floating point quaternion.
            bno055_quaternion_t result;
            constexpr float scale = 1.0 / (1 << 14);
            result.w = int16_t((uint16_t(buffer[1]) << 8) | uint16_t(buffer[0])) * scale;
            result.x = int16_t((uint16_t(buffer[3]) << 8) | uint16_t(buffer[2])) * scale;
            result.y = int16_t((uint16_t(buffer[5]) << 8) | uint16_t(buffer[4])) * scale;
            result.z = int16_t((uint16_t(buffer[7]) << 8) | uint16_t(buffer[6])) * scale;
            return result;
        }

        bno055_euler_angles_t getEulerAngles() {
            auto q = getQuaternion();
            // Convert the quaternion to Euler angles.
            bno055_euler_angles_t angles;
            float sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
            float cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
            angles.roll = atan2f(sinr_cosp, cosr_cosp);
            float sinp = +2.0 * (q.w * q.y - q.z * q.x);
            angles.pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asinf(sinp);
            float siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
            float cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            angles.yaw = atan2(siny_cosp, cosy_cosp);
            return angles;
        }

        bno055_vector_t getAngularVelocity() {
            uint8_t buffer[6];
            memset(buffer, 0x00, 6);
            readNBytes(bno055_register_t::BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
            // Translate raw values to floating point vector.
            bno055_vector_t result;
            constexpr float scale = 1. / 900.;
            result.x = int16_t((uint16_t(buffer[1]) << 8) | uint16_t(buffer[0])) * scale;
            result.y = int16_t((uint16_t(buffer[3]) << 8) | uint16_t(buffer[2])) * scale;
            result.z = int16_t((uint16_t(buffer[5]) << 8) | uint16_t(buffer[4])) * scale;
            return result;
        }

        bno055_vector_t getLinearAcceleration() {
            uint8_t buffer[6];
            memset(buffer, 0x00, 6);
            readNBytes(bno055_register_t::BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
            // Translate raw values to floating point vector.
            bno055_vector_t result;
            constexpr float scale = 1. / 100.;
            result.x = int16_t((uint16_t(buffer[1]) << 8) | uint16_t(buffer[0])) * scale;
            result.y = int16_t((uint16_t(buffer[3]) << 8) | uint16_t(buffer[2])) * scale;
            result.z = int16_t((uint16_t(buffer[5]) << 8) | uint16_t(buffer[4])) * scale;
            return result;
        }

        bool isFullyCalibrated() {
            auto calibration_status = getCalibrationStatus();
            return (calibration_status.system >= 3) && (calibration_status.accelerometer >= 3)
                   && (calibration_status.gyroscope >= 3) && (calibration_status.magnetometer >= 3);
        }

        bno055_calibration_status_t getCalibrationStatus() {
            bno055_calibration_status_t result;
            uint8_t calibration_status = readByte(bno055_register_t::BNO055_CALIB_STAT_ADDR);
            result.magnetometer = (calibration_status & 0x03);
            result.accelerometer = (calibration_status & 0x0c) >> 2;
            result.gyroscope = (calibration_status & 0x30) >> 4;
            result.system = (calibration_status & 0xc0) >> 6;
            return result;
        }

        void getCalibrationValues(bno055_calibration_values_t &calibration_values) {
            // Switch to config mode.
            auto previous_operation_mode
                = setOperationMode(bno055_operation_mode_t::OPERATION_MODE_CONFIG);
            delay(25);
            // Read raw calibration values.
            uint8_t buffer[22];
            memset(buffer, 0x00, 22);
            readNBytes(bno055_register_t::ACCEL_OFFSET_X_LSB_ADDR, buffer, 22);
            // Translate raw values to offsets.
            calibration_values.accel_offset_x = (uint16_t(buffer[1]) << 8) | uint16_t(buffer[0]);
            calibration_values.accel_offset_y = (uint16_t(buffer[3]) << 8) | uint16_t(buffer[2]);
            calibration_values.accel_offset_z = (uint16_t(buffer[5]) << 8) | uint16_t(buffer[4]);
            calibration_values.mag_offset_x = (uint16_t(buffer[7]) << 8) | uint16_t(buffer[6]);
            calibration_values.mag_offset_y = (uint16_t(buffer[9]) << 8) | uint16_t(buffer[8]);
            calibration_values.mag_offset_z = (uint16_t(buffer[11]) << 8) | uint16_t(buffer[10]);
            calibration_values.gyro_offset_x = (uint16_t(buffer[13]) << 8) | uint16_t(buffer[12]);
            calibration_values.gyro_offset_y = (uint16_t(buffer[15]) << 8) | uint16_t(buffer[14]);
            calibration_values.gyro_offset_z = (uint16_t(buffer[17]) << 8) | uint16_t(buffer[16]);
            calibration_values.accel_radius = (uint16_t(buffer[19]) << 8) | uint16_t(buffer[18]);
            calibration_values.mag_radius = (uint16_t(buffer[21]) << 8) | uint16_t(buffer[20]);
            // Restore previous operation mode.
            setOperationMode(previous_operation_mode);
        }

        void setCalibrationValues(const bno055_calibration_values_t &calibration_values) {
            // Switch to config mode.
            auto previous_operation_mode
                = setOperationMode(bno055_operation_mode_t::OPERATION_MODE_CONFIG);
            delay(25);
            // Translate offsets to raw calibration values.
            uint8_t buffer[22];
            buffer[0] = calibration_values.accel_offset_x & 0x00ff;
            buffer[1] = (calibration_values.accel_offset_x & 0xff00) >> 8;
            buffer[2] = calibration_values.accel_offset_y & 0x00ff;
            buffer[3] = (calibration_values.accel_offset_y & 0xff00) >> 8;
            buffer[4] = calibration_values.accel_offset_z & 0x00ff;
            buffer[5] = (calibration_values.accel_offset_z & 0xff00) >> 8;
            buffer[6] = calibration_values.mag_offset_x & 0x00ff;
            buffer[7] = (calibration_values.mag_offset_x & 0xff00) >> 8;
            buffer[8] = calibration_values.mag_offset_y & 0x00ff;
            buffer[9] = (calibration_values.mag_offset_y & 0xff00) >> 8;
            buffer[10] = calibration_values.mag_offset_z & 0x00ff;
            buffer[11] = (calibration_values.mag_offset_z & 0xff00) >> 8;
            buffer[12] = calibration_values.gyro_offset_x & 0x00ff;
            buffer[13] = (calibration_values.gyro_offset_x & 0xff00) >> 8;
            buffer[14] = calibration_values.gyro_offset_y & 0x00ff;
            buffer[15] = (calibration_values.gyro_offset_y & 0xff00) >> 8;
            buffer[16] = calibration_values.gyro_offset_z & 0x00ff;
            buffer[17] = (calibration_values.gyro_offset_z & 0xff00) >> 8;
            buffer[18] = calibration_values.accel_radius & 0x00ff;
            buffer[19] = (calibration_values.accel_radius & 0xff00) >> 8;
            buffer[20] = calibration_values.mag_radius & 0x00ff;
            buffer[21] = (calibration_values.mag_radius & 0xff00) >> 8;
            // Write calibration_values to sensor.
            writeNBytes(bno055_register_t::ACCEL_OFFSET_X_LSB_ADDR, buffer, 22);
            // Restore previous operation mode.
            setOperationMode(previous_operation_mode);
        }

      private:
        void writeByte(bno055_register_t sensor_register, uint8_t value) {
            i2c_device.beginTransmission(i2c_address);
            i2c_device.write(uint8_t(sensor_register));
            i2c_device.write(value);
            i2c_device.endTransmission(I2C_STOP, 0);
        }

        void writeNBytes(bno055_register_t sensor_register, uint8_t *buffer, uint8_t size) {
            i2c_device.beginTransmission(i2c_address);
            for(uint8_t i = 0; i < size; ++i) {
                i2c_device.write(uint8_t(sensor_register) + i);
                i2c_device.write(buffer[i]);
            }
            i2c_device.endTransmission(I2C_STOP, 0);
        }

        uint8_t readByte(bno055_register_t sensor_register) {
            uint8_t value = 0u;
            i2c_device.beginTransmission(i2c_address);
            i2c_device.write(uint8_t(sensor_register));
            i2c_device.endTransmission(I2C_STOP, 0);
            i2c_device.requestFrom(i2c_address, uint8_t(1u));
            value = i2c_device.receive();
            return value;
        }

        void readNBytes(bno055_register_t sensor_register, uint8_t *buffer, uint8_t size) {
            i2c_device.beginTransmission(i2c_address);
            i2c_device.write(uint8_t(sensor_register));
            i2c_device.endTransmission(I2C_STOP, 0);
            i2c_device.requestFrom(i2c_address, size);
            for(uint8_t i = 0; i < size; ++i) {
                buffer[i] = Wire.read();
            }
        }

        bno055_operation_mode_t setOperationMode(bno055_operation_mode_t operation_mode) {
            auto previous_operation_mode = bno055_operation_mode;
            writeByte(bno055_register_t::BNO055_OPR_MODE_ADDR, uint8_t(operation_mode));
            bno055_operation_mode = operation_mode;
            delay(30);
            return previous_operation_mode;
        }

        void setPowerMode(bno055_power_mode_t power_mode) {
            writeByte(bno055_register_t::BNO055_PWR_MODE_ADDR, uint8_t(power_mode));
            delay(30);
        }
    };

}  // namespace earth_rover

#endif