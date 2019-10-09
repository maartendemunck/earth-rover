#ifndef __EARTH_ROVER_HMI__CAR_CONFIGURATION__
#define __EARTH_ROVER_HMI__CAR_CONFIGURATION__


#include "servo-configuration.hpp"
#include "radio-configuration.hpp"


namespace earth_rover_hmi
{

  class CarConfiguration
  {
    private:
      ServoConfiguration steering_configuration;  //!< Steering servo configuration.
      ServoConfiguration throttle_configuration;  //!< ESC configuration.
      ServoConfiguration gearbox_configuration;   //!< Gearbx servo configuration.
      RadioConfiguration radio_configuration;     //!< Radio configuration.

    public:
      CarConfiguration();
      CarConfiguration(const ServoConfiguration & steering_configuration,
                       const ServoConfiguration & throttle_configuration,
                       const ServoConfiguration & gearbox_configuration,
                       const RadioConfiguration & radio_configuration);
      CarConfiguration(ServoConfiguration && steering_configuration,
                       ServoConfiguration && throttle_configuration,
                       ServoConfiguration && gearbox_configuration,
                       RadioConfiguration && radio_configuration);

      void setup() { ; }

      void spinOnce() { ; }

      ServoConfiguration & getSteeringConfig() { return steering_configuration; };
      ServoConfiguration & getThrottleConfig() { return throttle_configuration; };
      ServoConfiguration & getGearboxConfig() { return gearbox_configuration; };
      RadioConfiguration & getRadioConfig() { return radio_configuration; };

  };

}

#endif