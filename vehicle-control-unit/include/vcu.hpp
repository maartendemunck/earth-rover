#ifndef __VCU__
#define __VCU__


#include <cstdint>
#include <i2c_t3.h>
#include "adafruit_bno055.hpp"
#include "configured-servo.hpp"
#include "lighting.hpp"
#include "position-encoder.hpp"


namespace earth_rover_vcu
{

  template<typename Steering_t, typename Powertrain_t, typename Gps_t>
  class Vcu
  {

    Lighting automotive_lighting;
    PositionEncoder<7u, 8u> position_encoder;  // TODO: parameterize pin numbers!
    i2c_t3 & bno055_imu_i2c_device;
    const uint8_t bno055_imu_i2c_scl_pin;
    const uint8_t bno055_imu_i2c_sda_pin;
    AdafruitBNO055<i2c_t3> bno055_imu_device;
    Steering_t & steering;      //!< Steering device driver.
    Powertrain_t & powertrain;  //!< Powertrain device driver.
    Gps_t & gps;                //!< GPS device driver.

    elapsedMillis since_last_control_message;
    bool timeout_handler_called;

    public:

      Vcu(uint8_t head_lamp_pin, uint8_t tail_lamp_pin, uint8_t turn_signal_right_pin, uint8_t turn_signal_left_pin,
        i2c_t3 & imu_i2c, uint8_t imu_i2c_scl_pin, uint8_t imu_i2c_sda_pin,
        Steering_t & steering, Powertrain_t & powertrain, Gps_t & gps)
      :
        automotive_lighting {head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin},
        position_encoder {23000u, 0u, 0u},
        bno055_imu_i2c_device {imu_i2c},
        bno055_imu_i2c_scl_pin {imu_i2c_scl_pin},
        bno055_imu_i2c_sda_pin {imu_i2c_sda_pin},
        bno055_imu_device {imu_i2c, AdafruitBNO055<i2c_t3>::BNO055_ADDRESS_A},
        steering {steering},
        powertrain {powertrain},
        gps {gps}
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
        automotive_lighting.setStopLamps(true);
        automotive_lighting.setHazardFlashers(true);
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
        automotive_lighting.spinOnce();
        gps.spinOnce();
      }

      void handleControlMessage(int16_t steering_angle, int16_t throttle, int8_t gearbox, int8_t lighting)
      {
        // Send settings to subsystems.
        steering.setNormalizedSteeringAngle(steering_angle);
        powertrain.setNormalizedThrottleSetting(throttle);
        powertrain.setGear(gearbox);
        // Set automotive lighting.
        automotive_lighting.setTurnSignalRight(lighting & 0x01);
        automotive_lighting.setTurnSignalLeft(lighting & 0x02);
        automotive_lighting.setDippedBeam(lighting & 0x04);
        automotive_lighting.setHighBeam(lighting & 0x08);
        automotive_lighting.setHazardFlashers(lighting & 0x10);
        automotive_lighting.setStopLamps(throttle > -50 && throttle < 50);
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
        automotive_lighting.setStopLamps(true);
        automotive_lighting.setHazardFlashers(true);
        // Timeout handler called.
        timeout_handler_called = true;
      }

  };

}

#endif