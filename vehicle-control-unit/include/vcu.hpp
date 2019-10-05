#ifndef __VCU__
#define __VCU__


#include <cstdint>
#include <i2c_t3.h>
#include "adafruit_bno055.hpp"


namespace earth_rover_vcu
{

  template<typename Steering_t, typename Powertrain_t, typename Lighting_t, typename PositionEncoder_t, typename Gps_t>
  class Vcu
  {
    private:

      Steering_t & steering;                 //!< Steering device driver.
      Powertrain_t & powertrain;             //!< Powertrain device driver.
      Lighting_t & lighting;                 //!< Automotive lighting device driver.
      PositionEncoder_t & position_encoder;  //!< Position encoder device driver.
      Gps_t & gps;                           //!< GPS device driver.

      i2c_t3 & bno055_imu_i2c_device;
      const uint8_t bno055_imu_i2c_scl_pin;
      const uint8_t bno055_imu_i2c_sda_pin;
      AdafruitBNO055<i2c_t3> bno055_imu_device;

      elapsedMillis since_last_control_message;  //!< Time since we received a control message.
      bool timeout_handler_called;               //!< Control message timeout called.

    public:

      Vcu(Steering_t & steering, Powertrain_t & powertrain, Lighting_t & lighting,
          PositionEncoder_t & position_encoder, Gps_t & gps,
          i2c_t3 & imu_i2c, uint8_t imu_i2c_scl_pin, uint8_t imu_i2c_sda_pin)
      :
        steering {steering},
        powertrain {powertrain},
        lighting {lighting},
        position_encoder {position_encoder},
        gps {gps},
        bno055_imu_i2c_device {imu_i2c},
        bno055_imu_i2c_scl_pin {imu_i2c_scl_pin},
        bno055_imu_i2c_sda_pin {imu_i2c_sda_pin},
        bno055_imu_device {imu_i2c, AdafruitBNO055<i2c_t3>::BNO055_ADDRESS_A}
      {
        ;
      }

      ~Vcu() = default;

      //! Initialize the VCU and all subsystems.
      void setup()
      {
        // Initialize all subsystems.
        steering.setup();
        powertrain.setup();
        // Start with stop lamps and hazard flashers on, until we receive something from the HMI.
        lighting.setStopLamps(true);
        lighting.setHazardFlashers(true);
        // Initialize position encoder.
        position_encoder.setup();
        // Initialize the Bosch BNO055 IMU. TODO: get I²C configuration as a parameter?
        bno055_imu_i2c_device.begin(I2C_MASTER, 0x00, bno055_imu_i2c_scl_pin, bno055_imu_i2c_sda_pin, I2C_PULLUP_EXT,
                                    400000, I2C_OP_MODE_ISR);
        bno055_imu_device.setup();
        // Initialize the GPS.
        gps.setup();
        // This is a safe state, no need to call the timeout handler.
        timeout_handler_called = true;
      }

      void spinOnce()
      {
        if(since_last_control_message > 500u && !timeout_handler_called)
        {
          handleTimeout();
        }
        steering.spinOnce();
        powertrain.spinOnce();
        lighting.spinOnce();
        gps.spinOnce();
      }

      void handleControlMessage(int16_t steering_angle, int16_t throttle_setting, int8_t gear, int8_t lighting_setting)
      {
        // Send settings to subsystems.
        steering.setNormalizedSteeringAngle(steering_angle);
        powertrain.setNormalizedThrottleSetting(throttle_setting);
        powertrain.setGear(gear);
        // Set automotive lighting.
        lighting.setTurnSignalRight(lighting_setting & 0x01);
        lighting.setTurnSignalLeft(lighting_setting & 0x02);
        lighting.setDippedBeam(lighting_setting & 0x04);
        lighting.setHighBeam(lighting_setting & 0x08);
        lighting.setHazardFlashers(lighting_setting & 0x10);
        lighting.setStopLamps(throttle_setting > -50 && throttle_setting < 50);
        // Reset control message timout.
        since_last_control_message = 0;
        timeout_handler_called = false;
      }

      inline int16_t getSpeed() const
      {
        return position_encoder.getSpeed();
      }

      inline uint64_t getOdometer() const
      {
        return position_encoder.getOdometer();
      }

      inline uint64_t getTripmeter() const
      {
        return position_encoder.getTripmeter();
      }

      AdafruitBNO055<i2c_t3>::bno055_euler_angles_t getOrientation()
      {
        // Get Euler angles.
        auto angles = bno055_imu_device.getEulerAngles();
        // Adjust Euler angles for the orientation of the Adafruit BNO055 IMU in the car.
        angles.yaw = - angles.yaw - 90.;
        while(angles.yaw < 0)
        {
          angles.yaw += 2. * M_PI;
        }
        angles.pitch = angles.pitch;
        angles.roll = - angles.roll;
        return angles;
      }

      AdafruitBNO055<i2c_t3>::bno055_calibration_status_t getImuCalibrationStatus()
      {
        return bno055_imu_device.getCalibrationStatus();
      }

      inline gps_fix getGpsData() const
      {
        return gps.getCurrentGpsFix();
      }

    private:

      void handleTimeout()
      {
        // Stop car and turn stop lamps and hazard flashers on.
        powertrain.setNormalizedThrottleSetting(0);
        lighting.setStopLamps(true);
        lighting.setHazardFlashers(true);
        // Timeout handler called.
        timeout_handler_called = true;
      }

  };

}

#endif