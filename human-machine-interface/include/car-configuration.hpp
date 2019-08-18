#ifndef __CAR_CONFIGURATION__
#define __CAR_CONFIGURATION__


#include "servo-configuration.hpp"
#include "radio-configuration.hpp"


namespace earth_rover
{

  class CarConfiguration
  {
    private:

      uint16_t car_id;
      ServoConfiguration steering_configuration;
      ServoConfiguration throttle_configuration;
      ServoConfiguration gearbox_configuration;
      RadioConfiguration radio_configuration;

    public:

      CarConfiguration(uint16_t car_id);
      CarConfiguration(uint16_t car_id,
                       const ServoConfiguration & steering_configuration,
                       const ServoConfiguration & throttle_configuration,
                       const ServoConfiguration & gearbox_configuration,
                       const RadioConfiguration & radio_configuration);
      CarConfiguration(uint16_t car_id,
                       ServoConfiguration && steering_configuration,
                       ServoConfiguration && throttle_configuration,
                       ServoConfiguration && gearbox_configuration,
                       RadioConfiguration && radio_configuration);

      uint16_t getCarId() { return car_id; };
      ServoConfiguration & getSteeringConfig() { return steering_configuration; };
      ServoConfiguration & getThrottleConfig() { return throttle_configuration; };
      ServoConfiguration & getGearboxConfig() { return gearbox_configuration; };
      RadioConfiguration & getRadioConfig() { return radio_configuration; };

  };

}

#endif