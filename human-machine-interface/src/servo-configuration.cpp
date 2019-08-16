#include "servo-configuration.hpp"


namespace earth_rover
{

  ServoConfiguration::ServoConfiguration(uint8_t input_channel):
    Configuration(),
    current_configuration {1000u, 1500u, 2000u, true, input_channel}
  {
    ;
  }


  ServoConfiguration::ServoConfiguration(const ServoConfiguration & configuration):
    Configuration(),
    current_configuration {configuration.current_configuration}
  {
    ;
  }


  ServoConfiguration::ServoConfiguration(
      uint16_t minimum, uint16_t center, uint16_t maximum, bool enforce, uint8_t input_channel):
    Configuration(),
    current_configuration {minimum, center, maximum, enforce, input_channel}
  {
    ;
  }


  void ServoConfiguration::setConfiguration(const Data & configuration, Changed change_source)
  {
    current_configuration = configuration;
    updateChanged(change_source);
  }


  void ServoConfiguration::setConfiguration(
      uint16_t minimum, uint16_t center, uint16_t maximum, bool enforce, uint8_t input_channel, Changed change_source)
  {
    current_configuration.minimum = minimum;
    current_configuration.center = center;
    current_configuration.maximum = maximum;
    current_configuration.enforce = enforce;
    current_configuration.input_channel = input_channel;
    updateChanged(change_source);
  }


  void ServoConfiguration::setMinimum(uint16_t minimum, Changed change_source)
  {
    current_configuration.minimum = minimum;
    updateChanged(change_source);
  }


  void ServoConfiguration::setCenter(uint16_t center, Changed change_source)
  {
    current_configuration.center = center;
    updateChanged(change_source);
  }


  void ServoConfiguration::setMaximum(uint16_t maximum, Changed change_source)
  {
    current_configuration.maximum = maximum;
    updateChanged(change_source);
  }


  void ServoConfiguration::setEnforce(bool enforce, Changed change_source)
  {
    current_configuration.enforce = enforce;
    updateChanged(change_source);
  }


  void ServoConfiguration::setInputChannel(uint8_t input_channel, Changed change_source)
  {
    current_configuration.input_channel = input_channel;
    updateChanged(change_source);
  }


  ServoConfiguration::Data ServoConfiguration::resetConfigurationChanged()
  {
    configuration_changed = false;
    return current_configuration;
  }


  ServoConfiguration::Data ServoConfiguration::resetDisplayChanged()
  {
    display_changed = false;
    return current_configuration;
  }

}