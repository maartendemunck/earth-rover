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

      template <typename T>
      struct SensorData
      {
        T data;
        bool valid;
        bool updated;
        elapsedMillis since_last_update;
      };

      SensorData<Speedometer> speedometer;
      SensorData<Orientation> orientation;
      SensorData<Location> location;
      SensorData<int32_t> altitude;

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

      void setSpeedometer(const Speedometer & new_speedometer, bool valid = true);
      bool isSpeedometerUpdated() { return speedometer.updated; }
      std::pair<bool, Speedometer> getSpeedometer(bool reset = true);
      void setOrientation(const Orientation & new_orientation, bool valid = true);
      bool isOrientationUpdated() { return orientation.updated; }
      std::pair<bool, Orientation> getOrientation(bool reset = true);
      void setLocation(const Location & new_location, bool valid = true);
      bool isLocationUpdated() { return location.updated; }
      std::pair<bool, Location> getLocation(bool reset = true);
      void setAltitude(int32_t new_altitude, bool valid = true);
      bool isAltitudeUpdated() { return altitude.updated; }
      std::pair<bool, int32_t> getAltitude(bool reset = true);

  };

}

#endif