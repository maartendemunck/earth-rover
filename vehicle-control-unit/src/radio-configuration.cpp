//! Radio (nRF24L01+) configuration (implementation).
/*!
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "radio-configuration.hpp"


namespace earth_rover_vcu
{

  bool RadioConfiguration::setHmiRadioPowerLevel(uint8_t power_level)
  {
    if(power_level >= 0 && power_level <= 3)
    {
      if(power_level != hmi_radio_power_level)
      {
        hmi_radio_power_level = power_level;
        changed = true;
      }
      return true;
    }
    else
    {
      return false;
    }
  }


  bool RadioConfiguration::setVcuRadioPowerLevel(uint8_t power_level)
  {
    if(power_level >= 0 && power_level <= 3)
    {
      if(power_level != vcu_radio_power_level)
      {
        vcu_radio_power_level = power_level;
        changed = true;
      }
      return true;
    }
    else
    {
      return false;
    }
  }


  bool RadioConfiguration::serialize(uint8_t * data, uint16_t size)
  {
    if(size >= 2u)
    {
      data[0] = hmi_radio_power_level;
      data[1] = vcu_radio_power_level;
      for(unsigned index = 2; index < size; ++ index)
      {
        data[index] = 0xff;
      }
      // Reset changed and save flags (we write this configuration to EEPROM, so changes are stored).
      changed = false;
      save_required = false;
      return true;
    }
    else
    {
      return false;
    }
  }


  bool RadioConfiguration::deserialize(uint8_t * data, uint16_t size)
  {
    if(size >= 2u)
    {
      hmi_radio_power_level = data[0];
      vcu_radio_power_level = data[1];
      // Reset changed and save flags (we got this configuration from EEPROM, so it's not a change).
      changed = false;
      save_required = false;
      return true;
    }
    else
    {
      return false;
    }
  }

}