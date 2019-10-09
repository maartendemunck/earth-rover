#ifndef __EARTH_ROVER_HMI__SERVO_CONFIGURATION__
#define __EARTH_ROVER_HMI__SERVO_CONFIGURATION__


#include <cstdint>
#include "configuration.hpp"


namespace earth_rover_hmi
{

  class ServoConfiguration: public Configuration
  {

    public:

      struct Data
      {

        uint16_t minimum;
        uint16_t center;
        uint16_t maximum;
        bool enforce;
        uint8_t input_channel;

        Data(uint16_t minimum, uint16_t center, uint16_t maximum, bool enforce, uint8_t input_channel):
          minimum {minimum},
          center {center},
          maximum {maximum},
          enforce {enforce},
          input_channel {input_channel}
        {
          ;
        }

      };

    private:

      Data current_configuration;

    public:

      ServoConfiguration(uint8_t input_channel);
      ServoConfiguration(const ServoConfiguration & configuration);
      ServoConfiguration(uint16_t minimum, uint16_t center, uint16_t maximum, bool enforce, uint8_t input_channel);

      inline Data getConfiguration() { return current_configuration; };
      inline uint16_t getMinimum() { return current_configuration.minimum; };
      inline uint16_t getCenter() { return current_configuration.center; };
      inline uint16_t getmaximum() { return current_configuration.maximum; };
      inline bool getEnforce() { return current_configuration.enforce; };
      inline uint8_t getInputChannel() { return current_configuration.input_channel; };
      void setConfiguration(const Data & configuration, Changed change_source);
      void setConfiguration(uint16_t minimum, uint16_t center, uint16_t maximum, bool enforce, uint8_t input_channel,
                            Changed change_source);
      void setMinimum(uint16_t minimum, Changed change_source);
      void setCenter(uint16_t center, Changed change_source);
      void setMaximum(uint16_t maximum, Changed change_source);
      void setEnforce(bool enforce, Changed change_source);
      void setInputChannel(uint8_t input_channel, Changed change_source);

      Data resetConfigurationChanged();
      Data resetDisplayChanged();

  };

}

#endif