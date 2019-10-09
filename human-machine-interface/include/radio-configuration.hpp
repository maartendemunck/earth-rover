#ifndef __EARTH_ROVER_HMI__RADIO_CONFIGURATION__
#define __EARTH_ROVER_HMI__RADIO_CONFIGURATION__


#include <cstdint>
#include "configuration.hpp"


namespace earth_rover_hmi
{

  class RadioConfiguration: public Configuration
  {
    public:

      struct Data
      {
        uint16_t tx_power;
        uint16_t rx_power;

        Data(uint16_t tx_power, uint16_t rx_power):
          tx_power {tx_power},
          rx_power {rx_power}
        {
          ;
        }

      };

    private:

      Data current_configuration;
      bool configuration_changed;
      bool display_changed;

    public:

      RadioConfiguration();
      RadioConfiguration(const RadioConfiguration & configuration);
      RadioConfiguration(uint16_t tx_power, uint16_t rx_power);

      Data getConfiguration() { return current_configuration; };
      inline uint16_t getTxPower() { return current_configuration.tx_power; };
      inline uint16_t getRxPower() { return current_configuration.rx_power; };
      void setConfiguration(const Data & configuration, Changed change_source);
      void setConfiguration(uint16_t tx_power, uint16_t rx_power, Changed change_source);
      void setTxPower(uint16_t tx_power, Changed change_source);
      void setRxPower(uint16_t rx_power, Changed change_source);

      Data resetConfigurationChanged();
      Data resetDisplayChanged();
  };

}

#endif