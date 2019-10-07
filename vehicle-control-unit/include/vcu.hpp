//! VCU (vehicle control unit) for the Earth Rover (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__VCU__
#define __EARTH_ROVER_VCU__VCU__


#include <cstdint>


namespace earth_rover_vcu
{

  //! VCU (vehicle control unit) for the Earth Rover.
  /*!
   *  The VCU is a rather abstract interface to the individual systems of the vehicle.
   * 
   *  \tparam Steering_t Actual type of the steering servo device driver.
   *  \tparam Powertrain_t Actual type of the powertrain device driver.
   *  \tparam Lighting_t Actual type of the automotive lighting device driver.
   *  \tparam PositionEncoder_t Actual type of the position encoder device driver.
   *  \tparam Imu_t Actual type of the IMU device driver.
   *  \tparam Gps_t Actual type of the GPS device driver.
   */
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

      //! Constructor.
      /*!
       *  \param steering Configured device driver for the steering servo.
       *  \param powertrain Configured device driver for the powertrain.
       *  \param lighting Configured device driver for the automotive lighting.
       *  \param position_encoder Configured device driver for the position encoder.
       *  \param imu Configured device driver for the IMU.
       *  \param gps Configured device driver for the GPS.
       */
      Vcu(
        Steering_t & steering, Powertrain_t & powertrain, Lighting_t & lighting,
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

      //! Default constructor.
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

      //! Spinning loop.
      /*!
       *  The spinning loop calls the spinning loop of all subsystems and handles radio timeouts.
       */
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

      //! Process a control message.
      /*!
       *  \param steering_angle Normalized steering angle
       *                        (-1000 = maximum left ... 0 = center ... +1000 = maximum right).
       *  \param throttle_setting Normalized throttle setting
       *                          (-1000 = full speed backwards ... 0 = stop ... +1000 = full speed forward).
       *  \param gear Gear.
       *  \param lighting_setting Lighting state (bit 0: right turn signal, bit 1: left turn signal,
       *                          bit 2: dipped beam headlamps, bit 3: high beam headlamps, bit 4: hazard flashers).
       */
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

      //! Get the current speed of the vehicle.
      /*!
       *  \return The current speed of the vehicle (in m/h).
       */
      inline int16_t getSpeed() const
      {
        return position_encoder.getSpeed();
      }

      //! Get the current value of the odometer.
      /*!
       *  \return The current value of the odometer (in m).
       */
      inline uint64_t getOdometer() const
      {
        return position_encoder.getOdometer();
      }

      //! Get the current value of the trip meter.
      /*!
       *  \return The current value of the trip meter (in m).
       */
      inline uint64_t getTripmeter() const
      {
        return position_encoder.getTripmeter();
      }

      //! Get the current orientation of the vehicle.
      /*!
       *  \return The current orientation of the vehicle (Euler angles in radians).
       */
      EulerAngles_t getOrientation()
      {
        return imu.getOrientation();
      }

      //! Check whether the IMU is fully calibrated.
      /*!
       *  \return True if the IMU is fully calibrated, false if not.
       */
      bool isImuFullyCalibrated()
      {
        //return false;
        return imu.isFullyCalibrated();
      }

      //! Get the current GPS fix.
      /*!
       *  \return The current GPS fix (in NeoGPS gps-fix format).
       */
      inline gps_fix getGpsData() const
      {
        return gps.getCurrentGpsFix();
      }

    private:

      //! Set the vehicle in a safe state in case of radio transmission timouts.
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