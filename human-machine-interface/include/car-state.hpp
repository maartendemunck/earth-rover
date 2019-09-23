#ifndef __CAR_STATE__
#define __CAR_STATE__


#include <cstdint>
#include <DMS.h>
#include "limit-value.hpp"


namespace earth_rover
{

  class CarState
  {
    public:

      struct Drive
      {
        int16_t steering;
        int16_t throttle;
        int8_t gear;
      };

      struct Lighting
      {
        bool turn_signal_right;
        bool turn_signal_left;
        bool dipped_beam;
        bool high_beam;
        bool hazard_flashers;
      };

      struct Speedometer
      {
        float speed;
        float odometer;
        float tripmeter;
      };

      struct Orientation
      {
        float yaw;
        float pitch;
        float roll;
      };

      struct Location
      {
        DMS_t latitude;
        DMS_t longitude;
      };

    private:

      Drive drive;
      int8_t gearbox_locked = 0;  //!< Used to implement the hysteresis in the shift down and up commands.
      Lighting lighting;
      int8_t turn_signal_free {0};  //!< Used to implement the hysteresis to automatically shut off the turn signals.
      bool turn_signal_left_cancelled {false};  //!< Used to shut off the turn signal using the steering command.
      bool turn_signal_right_cancelled {false};  //!< used to shut off the turn signal using the steering command.

      Speedometer speedometer;
      bool speedometer_changed;

      Orientation orientation;
      bool orientation_changed;

      struct
      {
        Location data;
        bool valid;
        bool changed;
        elapsedMillis since_last_update;
      } location;

      struct
      {
        int32_t data;
        bool valid;
        bool changed;
        elapsedMillis since_last_update;
      } altitude;

    public:

      CarState();
      ~CarState() = default;

      void spinOnce();

      void setSteeringInput(int16_t steering);
      void setThrottleInput(int16_t throttle);
      void setGearboxInput(int16_t gearbox);
      const Drive & getDriveInputs() { return drive; };
      void setTurnSignalRight(bool state);
      bool getTurnSignalRightCancelled(bool reset = true);
      void setTurnSignalLeft(bool state);
      bool getTurnSignalLeftCancelled(bool reset = true);
      void setDippedBeam(bool state);
      void setHighBeam(bool state);
      void setHazardFlashers(bool state);
      const Lighting & getLighting() { return lighting; };
      void setOrientation(const Orientation & new_orientation);
      const Orientation & getOrientation(bool reset = true);
      bool getOrientationChanged() { return orientation_changed; };
      void setLocation(const Location & new_location, bool valid = true);
      bool isLocationChanged() { return location.changed; };
      std::pair<bool, Location> getLocation(bool reset = true);
      void setAltitude(int32_t new_altitude, bool valid = true);
      bool isAltitudeChanged() { return altitude.changed; };
      std::pair<bool, int32_t> getAltitude(bool reset = true);

  };

}

#endif