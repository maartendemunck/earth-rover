#include <utility>
#include "car-configuration.hpp"


namespace earth_rover
{

  CarConfiguration::CarConfiguration(uint16_t car_id):
    car_id {uint16_t(car_id & 0x3ffu)},
    steering_configuration {1000u, 1500u, 2000u, true, 1u},
    throttle_configuration {1000u, 1500u, 2000u, true, 4u},
    gearbox_configuration {1100u, 1500u, 1900u, true, 3u},
    radio_configuration {202u, 0u, 0u, 0u}
  {
    ;
  }


  CarConfiguration::CarConfiguration(uint16_t car_id,
                                     const ServoConfiguration & steering_configuration,
                                     const ServoConfiguration & throttle_configuration,
                                     const ServoConfiguration & gearbox_configuration,
                                     const RadioConfiguration & radio_configuration):
    car_id {uint16_t(car_id & 0x3ffu)},
    steering_configuration {steering_configuration},
    throttle_configuration {throttle_configuration},
    gearbox_configuration {gearbox_configuration},
    radio_configuration {radio_configuration}
  {
    ;
  }


  CarConfiguration::CarConfiguration(uint16_t car_id,
                                     ServoConfiguration && steering_configuration,
                                     ServoConfiguration && throttle_configuration,
                                     ServoConfiguration && gearbox_configuration,
                                     RadioConfiguration && radio_configuration):
    car_id {uint16_t(car_id & 0x3ffu)},
    steering_configuration {std::move(steering_configuration)},
    throttle_configuration {std::move(throttle_configuration)},
    gearbox_configuration {std::move(gearbox_configuration)},
    radio_configuration {std::move(radio_configuration)}
  {
    ;
  }

}