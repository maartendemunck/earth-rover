//! Adafruit Bosch BNO055 IMU device driver for the Earth Rover's VCU (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__ADAFRUIT_BNO055_WRAPPER__
#define __EARTH_ROVER_VCU__ADAFRUIT_BNO055_WRAPPER__


#include <cstdint>
#include "adafruit-bno055.hpp"


namespace earth_rover_vcu
{

  //! Struct to store an orientation in Euler angles.
  /*!
   *  \ingroup VCU
   */
  struct EulerAngles_t
  {
    float yaw;    //!< Yaw (rotation around the vertical axis).
    float pitch;  //!< Pitch (rotation around the transverse axis).
    float roll;   //!< Roll (rotation around the longitudinal axis).
  };


  //! Wrapper for an AdafruitBNO055 instance, providing a device-independent IMU interface.
  /*!
   *  \tparam I2CDevice_t I²C device type.
   * 
   *  \ingroup VCU
   */
  template <typename I2CDevice_t>
  class AdafruitBNO055Wrapper
  {

    private:

      I2CDevice_t & i2c_device;                   //!< I²C device used to communicatie with the Adafruit BNO055 IMU.
      const uint8_t i2c_scl_pin;                  //!< I/O pin used for the I²C SCL signal.
      const uint8_t i2c_sda_pin;                  //!< I/O pin used for the I²C SDA signal.
      AdafruitBNO055<I2CDevice_t> bno055_device;  //!< Adafruit BNO055 device driver.

    public:

      //! Constructor.
      /*!
       *  \param i2c_device I²C device used to communicatie with the Adafruit BNO055 IMU.
       *  \param i2c_scl_pin I/O pin used for the I²C SCL signal.
       *  \param i2c_sda_pin I/O pin used for the I²C SDA signal.
       */
      AdafruitBNO055Wrapper(I2CDevice_t & i2c_device, uint8_t i2c_scl_pin, uint8_t i2c_sda_pin)
      :
        i2c_device {i2c_device},
        i2c_scl_pin {i2c_scl_pin},
        i2c_sda_pin {i2c_sda_pin},
        bno055_device {i2c_device, decltype(bno055_device)::BNO055_ADDRESS_A}
      {
        ;
      }

      //! Default destructor.
      ~AdafruitBNO055Wrapper() = default;

      //! Initialize the I2C device and the Adafruit BNO055 IMU.
      void setup()
      {
        i2c_device.begin(I2C_MASTER, 0x00, i2c_scl_pin, i2c_sda_pin, I2C_PULLUP_EXT, 400000, I2C_OP_MODE_ISR);
        bno055_device.setup();
      }

      //! Spinning loop.
      /*!
       *  \internal
       *  The Adafruit BNO055's spinning loop does nothing.
       */
      void spinOnce()
      {
        ;
      }
  
      //! Get the orientation of the vehicle in Euler angles, corrected for the orientation of the IMU in the vehicle.
      /*!
       *  \return The orientation of the vehicle in Euler angles.
       */
      EulerAngles_t getOrientation()
      {
        EulerAngles_t angles;
        // Get Euler angles from the Adafruit BNO055 IMU.
        auto angles_raw = bno055_device.getEulerAngles();
        // Adjust Euler angles for the orientation of the Adafruit BNO055 IMU in the car.
        angles.yaw = - angles_raw.yaw - (M_PI / 2.);
        while(angles.yaw < 0)
        {
          angles.yaw += 2. * M_PI;
        }
        angles.pitch = angles_raw.pitch;
        angles.roll = - angles_raw.roll;
        return angles;
      }

      //! Check whether the Adafruit BNO055 IMU is fully calibrated.
      /*!
       *  \return True if the IMU is fully calibrated, false if not.
       */
      bool isFullyCalibrated()
      {
        auto calibration_status = bno055_device.getCalibrationStatus();
        return (calibration_status.system == 3 && calibration_status.accelerometer == 3
                && calibration_status.gyroscope == 3 && calibration_status.magnetometer == 3);
      }

      //! Get the (minimal) size of the configuration block.
      /*!
       *  \return The minimal size of the configuration block.
       */
      uint16_t getConfigurationSize()
      {
        return sizeof(decltype(bno055_device)::bno055_calibration_values_t) + 1;  // Calibration values plus checksum.
      }

      //! Save the configuration to a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is written.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is written; false if not (if the buffer isn't large enough).
       */
      bool saveConfiguration(uint8_t * data, uint16_t size)
      {
        if(size >= getConfigurationSize())
        {
          auto calibration_values = bno055_device.getCalibrationValues();
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
          uint8_t checksum {0};
          for(unsigned index = 0; index < 22; ++ index)
          {
            checksum ^= data[index];
          }
          data[22] = checksum;
          return true;
        }
        else
        {
          return false;
        }
      }

      //! Load the configuration from a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is read or applied.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is applied; false if not (buffer not large enough or checksum wrong).
       */
      bool loadConfiguration(uint8_t * data, uint16_t size)
      {
        if(size >= getConfigurationSize())
        {
          uint8_t checksum {0};
          for(unsigned index = 0; index < 22; ++ index)
          {
            checksum ^= data[index];
          }
          if(data[22] == checksum)
          {
            typename decltype(bno055_device)::bno055_calibration_values_t calibration_values;
            calibration_values.accel_offset_x = (uint16_t(data[1]) << 8) | uint16_t(data[0]);
            calibration_values.accel_offset_y = (uint16_t(data[3]) << 8) | uint16_t(data[2]);
            calibration_values.accel_offset_z = (uint16_t(data[5]) << 8) | uint16_t(data[4]);
            calibration_values.mag_offset_x = (uint16_t(data[7]) << 8) | uint16_t(data[6]);
            calibration_values.mag_offset_y = (uint16_t(data[9]) << 8) | uint16_t(data[8]);
            calibration_values.mag_offset_z = (uint16_t(data[11]) << 8) | uint16_t(data[10]);
            calibration_values.gyro_offset_x = (uint16_t(data[13]) << 8) | uint16_t(data[12]);
            calibration_values.gyro_offset_y = (uint16_t(data[15]) << 8) | uint16_t(data[14]);
            calibration_values.gyro_offset_z = (uint16_t(data[17]) << 8) | uint16_t(data[16]);
            calibration_values.accel_radius = (uint16_t(data[19]) << 8) | uint16_t(data[18]);
            calibration_values.mag_radius = (uint16_t(data[21]) << 8) | uint16_t(data[20]);
            bno055_device.setCalibrationValues(calibration_values);
            return true;
          }
          else
          {
            // Checksum wrong.
            return false;
          }
        }
        else
        {
          // Buffer not large enough.
          return false;
        }
      }

  };

}

#endif