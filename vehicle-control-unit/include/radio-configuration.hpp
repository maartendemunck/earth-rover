//! Radio (nRF24L01+) configuration (interface and inline part of the implementation).
/*!
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__RADIO_CONFIGURATION
#define __EARTH_ROVER_VCU__RADIO_CONFIGURATION


#include <cstdint>


namespace earth_rover_vcu
{

  //! Radio configuration for the Earth Rover.
  /*!
   *  The radio configuration is stored in a separate (this) class to uniformize storing and restoring configuration
   *  and calibration settings.
   * 
   *  \ingroup VCU
   */
  class RadioConfiguration
  {
    private:

      uint8_t hmi_radio_power_level {1u};  //!< PA power level of the HMI's radio.
      uint8_t vcu_radio_power_level {1u};  //!< PA power level of the VCU's radio.
      bool changed {false};                //!< True if the radio settings changed.
      bool save_required {false};          //!< True if the radio configuration should be written to EEPROM.

    public:

      //! Default constructor.
      RadioConfiguration() = default;

      //! Default destructor.
      ~RadioConfiguration() = default;

      //! Set the PA power level of the HMI's radio.
      /*!
       *  \param power_level PA power level of the HMI's radio.
       *  \return True if the specified power level is valid, false if it isn't.
       */
      bool setHmiRadioPowerLevel(uint8_t power_level);

      //! Get the PA power level of the HMI's radio.
      /*!
       *  \return The PA power level of the HMI's radio.
       */
      uint8_t getHmiRadioPowerLevel()
      {
        return hmi_radio_power_level;
      }

      //! Set the PA power level of the VCU's radio.
      /*!
       *  \param power_level PA power level of the VCU's radio.
       *  \return True if the specified power level is valid, false if it isn't.
       */
      bool setVcuRadioPowerLevel(uint8_t power_level);

      //! Get the PA power level of the VCU's radio.
      /*!
       *  \return The PA power level of the VCU's radio.
       */
      uint8_t getVcuRadioPowerLevel()
      {
        return vcu_radio_power_level;
      }

      //! Save the radio configuration to EEPROM.
      void saveConfiguration()
      {
        if(changed)
        {
          save_required = true;
        }
      }

      //! Check whether the radio configuration should be written to EEPROM.
      /*!
       *  \return True if the radio configuration should be written to EEPROM.
       */
      bool saveRequired()
      {
        return save_required;
      }

      //! Save the configuration to a buffer.
      /*!
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is written; false if not (if the buffer isn't large enough).
       */
      bool serialize(uint8_t * data, uint16_t size);

      //! Load the configuration from a buffer.
      /*!
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is applied; false if not (buffer not large enough or checksum wrong).
       */
      bool deserialize(uint8_t * data, uint16_t size);

  };

}

#endif