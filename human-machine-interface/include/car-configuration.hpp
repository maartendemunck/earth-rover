#ifndef __CAR_CONFIGURATION__
#define __CAR_CONFIGURATION__


#include "servo-configuration.hpp"
#include "radio-configuration.hpp"


namespace earth_rover
{

  class CarConfiguration
  {
    private:

      ServoConfiguration steering_servo_configuration;
      ServoConfiguration speed_controller_configuration;
      ServoConfiguration gearbox_servo_configuration;
      RadioConfiguration radio_configuration;

    public:

      CarConfiguration();
      CarConfiguration(const ServoConfiguration & steering_servo_configuration,
                       const ServoConfiguration & speed_controller_configuration,
                       const ServoConfiguration & gearbox_servo_configuration,
                       const RadioConfiguration & radio_configuration);
      CarConfiguration(ServoConfiguration && steering_servo_configuration,
                       ServoConfiguration && speed_controller_configuration,
                       ServoConfiguration && gearbox_servo_configuration,
                       RadioConfiguration && radio_configuration);

      ServoConfiguration & getSteeringServoConfiguration() { return steering_servo_configuration; };
      ServoConfiguration & getSpeedControllerConfiguration() { return speed_controller_configuration; };
      ServoConfiguration & getGearboxServoConfiguration() { return gearbox_servo_configuration; };
      RadioConfiguration & getRadioConfiguration() { return radio_configuration; };

  };

}

#endif