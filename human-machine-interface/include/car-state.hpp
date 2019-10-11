//! Car state (digital twin) for the Earth Rover (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__CAR_STATE__
#define __EARTH_ROVER_HMI__CAR_STATE__


#include <cstdint>
#include <DMS.h>
#include "sensor-state.hpp"
#include "limit-value.hpp"


namespace
{
  //! Compare two DMS_t objects.
  /*!
   *  \param lhs Left hand side operand.
   *  \param rhs Right hand size operand.
   *  \return True if both objects are different, false if they are equal.
   * 
   *  \ingroup HMI
   */
  bool operator!= (const DMS_t & lhs, const DMS_t & rhs)
  {
    return (lhs.degrees != rhs.degrees || lhs.minutes != rhs.minutes || lhs.hemisphere != rhs.hemisphere
            || lhs.seconds_whole != rhs.seconds_whole || lhs.seconds_frac != rhs.seconds_frac);
  }
}


namespace earth_rover_hmi
{

  //! Earth Rover car state (digital twin).
  /*!
   *  This class models the current state of the Earth Rover.
   * 
   *  \ingroup HMI
   */
  class CarState
  {
    public:

      //! Steering and powertrain state.
      /*!
       *  \ingroup HMI
       */
      struct Drive
      {
        int16_t steering;  //!< Normalized steering angle (-1000...0...1000).
        int16_t throttle;  //!< Normalized throttle setting (-1000...0...1000).
        int8_t gear;       //!< Gear.
      };

      //! Automotive lighting state.
      /*!
       *  \ingroup HMI
       */
      struct Lighting
      {
        bool turn_signal_right;  //!< Right turn signal.
        bool turn_signal_left;   //!< Left turn signal.
        bool dipped_beam;        //!< Dipped beam headlamps.
        bool high_beam;          //!< High beam headlamps.
        bool hazard_flashers;    //!< Hazard flashers.
      };

      //! Speedometer measurement.
      /*
       *  \ingroup HMI
       */
      struct Speedometer
      {
        float speed;      //!< Speed (in km/h).
        float odometer;   //!< Odometer (in km).
        float tripmeter;  //!< Trip meter (in km).
        //! Overloaded comparison operator.
        /*!
         *  \param rhs Right hand side operand.
         *  \return true if both operands are different, false if they are equal.
         */
        bool operator!= (const Speedometer & rhs)
        {
          return (speed != rhs.speed || odometer != rhs.odometer || tripmeter != rhs.tripmeter);
        }
      };

      //! IMU measurement.
      /*!
       *  \ingroup HMI
       */
      struct Orientation
      {
        float yaw;    //!< Yaw (rotation around the vertical axis) (in degrees).
        float pitch;  //!< Pitch (rotation around the transverse axis) (in degrees).
        float roll;   //!< Roll (rotation around the longitudinal axis) (in degrees).
        //! Overloaded comparison operator.
        /*!
         *  \param rhs Right hand side operand.
         *  \return true if both operands are different, false if they are equal.
         */
        bool operator!= (const Orientation & rhs)
        {
          return (yaw != rhs.yaw || pitch != rhs.pitch || roll != rhs.roll);
        }
      };

      //! GPS location measurement.
      /*!
       *  \ingroup HMI
       */
      struct Location
      {
        DMS_t latitude;   //!< Latitude.
        DMS_t longitude;  //!< Longitude.
        //! Overloaded comparison operator.
        /*!
         *  \param rhs Right hand side operand.
         *  \return true if both operands are different, false if they are equal.
         */
        bool operator!= (const Location & rhs)
        {
          return (latitude != rhs.latitude || longitude != rhs.longitude);
        }
      };

    private:

      //! Current steering and powertrain state.
      Drive drive;
      //! Flag used to implement the hysteresis in the shift down and up commands.
      int8_t gearbox_locked = 0;
      //! Current state of the automotive lighting.
      Lighting lighting;
      //! Flag used to implement the hysteresis to automatically shut off the turn signals.
      int8_t turn_signal_free {0};
      //! Flag used to shut off the left turn signal using the steering command.
      bool turn_signal_left_cancelled {false};
      //! Flag used to shut off the right turn signal using the steering command.
      bool turn_signal_right_cancelled {false};

      //! Speed, odometer and trip meter measurement.
      SensorState<Speedometer> speedometer;
      //! Orientation measurement.
      SensorState<Orientation> orientation;
      //! GPS location (latitude and longitude) measurement.
      SensorState<Location> location;
      //! GPS altitude measurement.
      SensorState<int32_t> altitude;

    public:

      //! Constructor.
      CarState();

      //! Default destructor.
      ~CarState() = default;

      //! Initialize the car state.
      void setup()
      {
        ;
      }

      //! Spinning loop.
      void spinOnce()
      { 
        ;
      }

      //! Set the steering input.
      /*!
       *  \param steering Steering input (-1000...0...+1000).
       */
      void setSteeringInput(int16_t steering);

      //! Set the throttle input.
      /*!
       *  \param throttle Throttle input (-1000...0...+1000).
       */
      void setThrottleInput(int16_t throttle);

      //! Set the gearbox input.
      /*!
       *  \param gearbox Gearbox input (-1000...0...+1000).
       */
      void setGearboxInput(int16_t gearbox);

      //! Get requested steering and powertrain state.
      /*!
       *  \return Requested steering and powertrain state to communicate to the VCU.
       */
      const Drive & getRequestedDriveState()
      {
        return drive;
      }

      //! Set right turn signal input.
      /*!
       *  \param state Right turn signal input.
       */
      void setTurnSignalRight(bool state);

      //! Check whether the right turn signal is cancelled by steering inputs.
      /*!
       *  \param reset True to reset the cancelled flag, false to keep the current state of the cancelled flag.
       *  \return True if the turn signal was cancelled since the last time this function was called with reset = true).
       */
      bool getTurnSignalRightCancelled(bool reset = true);

      //! Set left turn signal input.
      /*!
       *  \param state Left turn signal input.
       */
      void setTurnSignalLeft(bool state);

      //! Check whether the left turn signal is cancelled by steering inputs.
      /*!
       *  \param reset True to reset the cancelled flag, false to keep the current state of the cancelled flag.
       *  \return True if the turn signal was cancelled since the last time this function was called with reset = true).
       */
      bool getTurnSignalLeftCancelled(bool reset = true);

      //! Set the dipped beam headlamps input.
      /*!
       *  \param state Dipped beam headlamps input.
       */
      void setDippedBeam(bool state);

      //! Set the high beam headlamps input.
      /*!
       *  \param state High beam headlamps input.
       */
      void setHighBeam(bool state);

      //! Set the hazard flashers input.
      /*!
       *  \param state Hazard flashers input.
       */
      void setHazardFlashers(bool state);

      //! Get requested automotive lighting state.
      /*!
       *  \return Requested automotive lighting state to communicate to the VCU.
       */
      const Lighting & getRequestedLightingState()
      {
        return lighting;
      }

      //! Set speedometer, odometer and trip meter measurements.
      /*!
       *  \param value Measurements.
       *  \param valid True if the last measurements are valid, false of not.
       */
      void setSpeedometer(const Speedometer & value, bool valid = true)
      {
        speedometer.set(value, valid);
      }

      //! Check whether the speedometer, odometer and tripmeter are updated.
      /*!
       *  Check whether the speedometer, odometer or tripmeter are updated since the last getSpeedometer(reset = true)
       *  call.
       *
       *  \return True if the speedometer, odometer or trip meter were updated, false if not.
       */
      bool isSpeedometerUpdated()
      {
        return speedometer.isUpdated();
      }

      //! Get the current speedometer, odometer and trip meter measurements.
      /*!
       *  \param reset_updated True to reset the updated flag, false to keep it.
       *  \return A pair with a flag indicating whether the current measurements are valid and the measurements itself.
       */
      auto getSpeedometer(bool reset_updated = true)
      {
        return speedometer.get(reset_updated);
      }

      //! Set orientation measurements.
      /*!
       *  \param value Measurements.
       *  \param valid True if the last measurements are valid, false of not.
       */
      void setOrientation(const Orientation & value, bool valid = true)
      {
        orientation.set(value, valid);
      }

      //! Check whether the orientation is updated.
      /*!
       *  Check whether the orientation is updated since the last getOrientation(reset = true) call.
       *
       *  \return True if the orientation was updated, false if not.
       */
      bool isOrientationUpdated()
      {
        return orientation.isUpdated();
      }

      //! Get the current orientation measurements.
      /*!
       *  \param reset_updated True to reset the updated flag, false to keep it.
       *  \return A pair with a flag indicating whether the current measurements are valid and the measurements itself.
       */
      auto getOrientation(bool reset_updated = true)
      {
        return orientation.get(reset_updated);
      }

      //! Set GPS location (latitude, longitude) measurements.
      /*!
       *  \param value Measurements.
       *  \param valid True if the last measurements are valid, false of not.
       */
      void setLocation(const Location & value, bool valid = true)
      {
        location.set(value, valid);
      }

      //! Check whether the GPS location (latitude, longitude) is updated.
      /*!
       *  Check whether the GPS location is updated since the last getLocation(reset = true) call.
       *
       *  \return True if the GPS location was updated, false if not.
       */
      bool isLocationUpdated()
      {
        return location.isUpdated();
      }

      //! Get the current GPS location (latitude, longitude) measurements.
      /*!
       *  \param reset_updated True to reset the updated flag, false to keep it.
       *  \return A pair with a flag indicating whether the current measurements are valid and the measurements itself.
       */
      auto getLocation(bool reset_updated = true)
      {
        return location.get(reset_updated);
      }

      //! Set GPS altitude measurement.
      /*!
       *  \param value Measurement.
       *  \param valid True if the last measurement is valid, false of not.
       */
      void setAltitude(int32_t value, bool valid = true)
      {
        altitude.set(value, true);
      }

      //! Check whether the GPS altitude is updated.
      /*!
       *  Check whether the GPS altitude is updated since the last getAltitude(reset = true) call.
       *
       *  \return True if the GPS altitude was updated, false if not.
       */
      bool isAltitudeUpdated()
      {
        return altitude.isUpdated();
      }

      //! Get the current GPS altitude measurement.
      /*!
       *  \param reset_updated True to reset the updated flag, false to keep it.
       *  \return A pair with a flag indicating whether the current measurement is valid and the measurement itself.
       */
      auto getAltitude(bool reset_updated = true)
      {
        return altitude.get(reset_updated);
      }

  };

}

#endif