#include <utility>
#include "car-configuration.hpp"


namespace earth_rover
{

  CarConfiguration::CarConfiguration():
    steering_servo_configuration {1000u, 1500u, 2000u, true, 1u},
    speed_controller_configuration {1000u, 1500u, 2000u, true, 4u},
    gearbox_servo_configuration {1100u, 1500u, 1900u, true, 3u},
    radio_configuration {202u, 0u, 0u, 0u}
  {
    ;
  }


  CarConfiguration::CarConfiguration(
      const ServoConfiguration & steering_servo_configuration,
      const ServoConfiguration & speed_controller_configuration,
      const ServoConfiguration & gearbox_servo_configuration,
      const RadioConfiguration & radio_configuration):
    steering_servo_configuration {steering_servo_configuration},
    speed_controller_configuration {speed_controller_configuration},
    gearbox_servo_configuration {gearbox_servo_configuration},
    radio_configuration {radio_configuration}
  {
    ;
  }


  CarConfiguration::CarConfiguration(
      ServoConfiguration && steering_servo_configuration,
      ServoConfiguration && speed_controller_configuration,
      ServoConfiguration && gearbox_servo_configuration,
      RadioConfiguration && radio_configuration):
    steering_servo_configuration {std::move(steering_servo_configuration)},
    speed_controller_configuration {std::move(speed_controller_configuration)},
    gearbox_servo_configuration {std::move(gearbox_servo_configuration)},
    radio_configuration {std::move(radio_configuration)}
  {
    ;
  }

}