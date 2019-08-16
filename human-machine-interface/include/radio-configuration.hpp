#ifndef __RADIO_CONFIGURATION__
#define __RADIO_CONFIGURATION__


#include <cstdint>
#include "configuration.hpp"


namespace earth_rover
{

  class RadioConfiguration: public Configuration
  {

    public:

      struct Data
      {

        uint16_t id;
        uint16_t tx_power;
        uint16_t rx_power;
        uint16_t channel;

        Data(uint16_t id, uint16_t tx_power, uint16_t rx_power, uint16_t channel):
          id {id},
          tx_power {tx_power},
          rx_power {rx_power},
          channel {channel}
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
      RadioConfiguration(uint16_t id, uint16_t tx_power, uint16_t rx_power, uint16_t channel);

      Data getConfiguration() { return current_configuration; };
      inline uint16_t getId() { return current_configuration.id; };
      inline uint16_t getTxPower() { return current_configuration.tx_power; };
      inline uint16_t getRxPower() { return current_configuration.rx_power; };
      inline uint16_t getChannel() { return current_configuration.channel; };
      void setConfiguration(const Data & configuration, Changed change_source);
      void setConfiguration(uint16_t id, uint16_t tx_power, uint16_t rx_power, uint16_t channel, Changed change_source);
      void setId(uint16_t id, Changed change_source);
      void setTxPower(uint16_t tx_power, Changed change_source);
      void setRxPower(uint16_t rx_power, Changed change_source);
      void setChannel(uint16_t channel, Changed change_source);

      Data resetConfigurationChanged();
      Data resetDisplayChanged();

  };

}

#endif