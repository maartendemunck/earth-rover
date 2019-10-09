#include "radio-configuration.hpp"


namespace earth_rover_hmi
{

  RadioConfiguration::RadioConfiguration():
    current_configuration {0u, 0u},
    configuration_changed {false},
    display_changed {false}
  {
    ;
  }


  RadioConfiguration::RadioConfiguration(const RadioConfiguration & configuration):
    current_configuration {configuration.current_configuration},
    configuration_changed {false},
    display_changed {false}
  {
    ;
  }


  RadioConfiguration::RadioConfiguration(uint16_t tx_power, uint16_t rx_power):
    current_configuration {tx_power, rx_power},
    configuration_changed {false},
    display_changed {false}
  {
    ;
  }


  void RadioConfiguration::setConfiguration(const Data & configuration, Changed change_source)
  {
    current_configuration = configuration;
    updateChanged(change_source);
  }


  void RadioConfiguration::setConfiguration(uint16_t tx_power, uint16_t rx_power, Changed change_source)
  {
    current_configuration.tx_power = tx_power;
    current_configuration.rx_power = rx_power;
    updateChanged(change_source);
  }


  void RadioConfiguration::setTxPower(uint16_t tx_power, Changed change_source)
  {
    current_configuration.tx_power = tx_power;
    updateChanged(change_source);
  }


  void RadioConfiguration::setRxPower(uint16_t rx_power, Changed change_source)
  {
    current_configuration.rx_power = rx_power;
    updateChanged(change_source);
  }


  RadioConfiguration::Data RadioConfiguration::resetConfigurationChanged()
  {
    configuration_changed = false;
    return current_configuration;
  }


  RadioConfiguration::Data RadioConfiguration::resetDisplayChanged()
  {
    display_changed = false;
    return current_configuration;
  }

}