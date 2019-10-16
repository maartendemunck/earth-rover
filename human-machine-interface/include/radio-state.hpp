//! Radio state (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__RADIO_STATE__
#define __EARTH_ROVER_HMI__RADIO_STATE__


#include <cstdint>
#include "configuration-parameter.hpp"
#include "limit-value.hpp"


namespace earth_rover_hmi
{

  //! Radio configuration.
  /*!
    *  \ingroup HMI
    */
  class RadioConfigParams
  {
    public:

      //! HMI radio power setting.
      uint8_t tx_power;
      //! VCU radio power setting.
      uint8_t rx_power;

      //! Create a radio configuration.
      /*!
       *  \param tx_power HMI radio power setting.
       *  \param rx_power VCU radio power setting.
       */
      RadioConfigParams(uint8_t tx_power, uint8_t rx_power)
      :
        tx_power {tx_power},
        rx_power {rx_power}
      {
        ;
      }

      //! Compare two radio configurations.
      /*!
       *  \param rhs Right hand side operand.
       *  \return True if both configurations are different, false if they are equal.
       */
      bool operator!= (const RadioConfigParams & rhs)
      {
        return (tx_power != rhs.tx_power || rx_power != rhs.rx_power);
      }

  };


  //! State of the HMI and VCU radios.
  /*!
   *  \ingroup HMI
   */
  class RadioState
  {
    private:

      //!< Radio configuration.
      ConfigurationParameter<RadioConfigParams> configuration;

    public:

      //! Constructor.
      /*!
       *  \param default_configuration Default configuration.
       */
      RadioState(RadioConfigParams default_configuration)
      :
        configuration {std::move(default_configuration)}
      {
        ;
      }

      //! Default destructor.
      ~RadioState() = default;

      //! Set the configuration stored in the VCU.
      /*!
       *  \param new_configuration Configuration stored in the VCU to store in our State object.
       */
      void setStoredConfiguration(const RadioConfigParams & new_configuration)
      {
        configuration.setStoredValue(new_configuration);
      }

      //! Check whether the configuration stored in the VCU is available to us.
      /*!
       *  \return True if the configuration stored in the VCU is available to us, false if it isn't.
       */
      bool isStoredConfigurationAvailable()
      {
        return configuration.isAvailable();
      }

      //! Set the new configuration.
      /*!
       *  \param new_configuration New configuration.
       */
      void setCurrentConfiguration(const RadioConfigParams & new_configuration)
      {
        return configuration.setCurrentValue(new_configuration);
      }
      
      //! Get the current configuration.
      /*!
       *  \return A const reference to the current configuration.
       */
      const RadioConfigParams & getCurrentConfiguration()
      {
        return configuration.getCurrentValue();
      }

      //! Check whether the current configuration is stored in non-volatile memory.
      /*!
       *  \return True if the current configuration is stored, false if not.
       */
      bool isCurrentConfigurationStored()
      {
        return configuration.isCurrentValueStored();
      }

  };

}

#endif