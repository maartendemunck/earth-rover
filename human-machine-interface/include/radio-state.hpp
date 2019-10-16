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
  using RadioState = ConfigurationParameter<RadioConfigParams>;

}

#endif