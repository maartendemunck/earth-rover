#ifndef __EARTH_ROVER_VCU__VCU__
#define __EARTH_ROVER_VCU__VCU__


#include <cstdint>


namespace earth_rover_vcu
{

  template<typename Steering_t, typename Powertrain_t, typename Lighting_t,
           typename PositionEncoder_t, typename Imu_t, typename Gps_t>
  class Vcu
  {
    private:

      Steering_t & steering;                     //!< Steering device driver.
      Powertrain_t & powertrain;                 //!< Powertrain device driver.
      Lighting_t & lighting;                     //!< Automotive lighting device driver.
      PositionEncoder_t & position_encoder;      //!< Position encoder device driver.
      Imu_t & imu;                               //!< IMU device driver.
      Gps_t & gps;                               //!< GPS device driver.

      elapsedMillis since_last_control_message;  //!< Time since we received a control message.
      bool timeout_handler_called;               //!< Control message timeout called.

    public:

      Vcu(Steering_t & steering, Powertrain_t & powertrain, Lighting_t & lighting,
          PositionEncoder_t & position_encoder, Imu_t & imu, Gps_t & gps)
      :
        steering {steering},
        powertrain {powertrain},
        lighting {lighting},
        position_encoder {position_encoder},
        imu {imu},
        gps {gps}
      {
        ;
      }

      ~Vcu() = default;

      //! Initialize the VCU and all subsystems.
      void setup()
      {
        // Initialize steering.
        steering.setup();
        // Initialize powertrain.
        powertrain.setup();
        // Start with stop lamps and hazard flashers on (until we receive our first command from the HMI).
        lighting.setup();
        lighting.setStopLamps(true);
        lighting.setHazardFlashers(true);
        // Initialize position encoder.
        position_encoder.setup();
        // Initialize the IMU.
        imu.setup();
        // Initialize the GPS.
        gps.setup();
        // This is a safe state, no need to call the timeout handler before we receive the first control message.
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
        position_encoder.spinOnce();
        imu.spinOnce();
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

      EulerAngles_t getOrientation()
      {
        return imu.getOrientation();
      }

      bool isImuFullyCalibrated()
      {
        //return false;
        return imu.isFullyCalibrated();
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