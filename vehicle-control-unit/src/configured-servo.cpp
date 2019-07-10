#include "configured-servo.hpp"
#include <algorithm>


namespace earth_rover
{

  ConfiguredServo::ConfiguredServo(
    uint8_t pin_number)
  :
    servo {},
    configuration {pin_number,
                   default_minimum_pulse_width, default_maximum_pulse_width, default_center_pulse_width,
                   default_initial_pulse_width,
                   default_enforce_pulse_width_limits},
    current_pulse_width {configuration.initial_pulse_width}
  {
    ;
  }


  ConfiguredServo::~ConfiguredServo()
  {
    servo.writeMicroseconds(configuration.initial_pulse_width);
    servo.detach();
  }


  void ConfiguredServo::setup()
  {
    servo.attach(configuration.pin_number);
    setPulseWidth(current_pulse_width);
  }


  void ConfiguredServo::setup(
    const Configuration & new_configuration)
  {
    configuration = new_configuration;
    setup();
  }


  void ConfiguredServo::setPosition(
    int16_t position)
  {
    if(position <= -1000)
    {
      current_pulse_width = configuration.minimum_pulse_width;
    }
    else if(position == 0)
    {
      current_pulse_width = configuration.center_pulse_width;
    }
    else if(position >= 1000)
    {
      current_pulse_width = configuration.maximum_pulse_width;
    }
    else if(position < 0)
    {
      current_pulse_width =
        configuration.center_pulse_width
        + int16_t(configuration.center_pulse_width - configuration.minimum_pulse_width) * position / 1000;
    }
    else  // position > 0)
    {
      current_pulse_width =
        configuration.center_pulse_width
        + int16_t(configuration.maximum_pulse_width - configuration.center_pulse_width) * position / 1000;
    }
    servo.writeMicroseconds(current_pulse_width);
  }


  void ConfiguredServo::setPulseWidth(
    uint16_t pulse_width)
  {
    current_pulse_width = correctPulseWidth(pulse_width);
    servo.writeMicroseconds(current_pulse_width);
  }


  void ConfiguredServo::setDefaultConfiguration()
  {
    configuration.minimum_pulse_width = default_minimum_pulse_width;
    configuration.maximum_pulse_width = default_maximum_pulse_width;
    configuration.center_pulse_width = default_center_pulse_width;
    configuration.initial_pulse_width = default_initial_pulse_width;
    configuration.enforce_pulse_width_limits = default_enforce_pulse_width_limits;
    if(configuration.enforce_pulse_width_limits)
    {
      setPulseWidth(current_pulse_width);
    }
  }


  void ConfiguredServo::setConfiguration(
    const ConfiguredServo::Configuration & new_configuration)
  {
    configuration = new_configuration;
    if(configuration.enforce_pulse_width_limits)
    {
      setPulseWidth(current_pulse_width);
    }
  }


  ConfiguredServo::Configuration ConfiguredServo::getConfiguration()
  const
  {
    return configuration;
  }


  uint16_t ConfiguredServo::correctPulseWidth(
    uint16_t requested_pulse_width)
  const
  {
    if(configuration.enforce_pulse_width_limits)
    {
      return std::max(configuration.minimum_pulse_width,
                      std::min(configuration.maximum_pulse_width, requested_pulse_width));
    }
    else
    {
      return requested_pulse_width;
    }
  }

}