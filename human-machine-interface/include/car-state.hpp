#ifndef __CAR_STATE__
#define __CAR_STATE__


#include <cstdint>
#include "limit-value.hpp"


namespace earth_rover
{

  class CarState
  {
    public:

      struct Drive {
        int16_t steering;
        int16_t throttle;
        int8_t gear;
      };

      struct Lighting {
        bool turn_signal_right;
        bool turn_signal_left;
        bool dipped_beam;
        bool high_beam;
        bool hazard_flashers;
      };

      struct Speedometer {
        float speed;
        float odometer;
        float tripmeter;
      };

      struct Orientation {
        float yaw;
        float pitch;
        float roll;
      };

      struct Location {
        float latitude;
        float longitude;
        float altitude;
      };

    private:

      Drive drive;
      int8_t gearbox_locked = 0;  //!< Used to implement the hysteresis in the shift down and up commands.
      Lighting lighting;
      bool lighting_changed;

      Speedometer speedometer;
      bool speedometer_changed;
      Orientation orientation;
      bool orientation_changed;
      Location location;
      bool location_changed;

    public:

      CarState();

      void setSteeringInput(int16_t steering);
      void setThrottleInput(int16_t throttle);
      void setGearboxInput(int16_t gearbox);
      const Drive & getDriveInputs() { return drive; };
      void setTurnSignalRight(bool state);
      void setTurnSignalLeft(bool state);
      void setDippedBeam(bool state);
      void setHighBeam(bool state);
      void setHazardFlashers(bool state);

  };

}

#endif