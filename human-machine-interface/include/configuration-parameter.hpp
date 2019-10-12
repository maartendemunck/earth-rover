//! Container for a configuration parameter (interface and template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__CONFIGURATION_PARAMETER__
#define __EARTH_ROVER_HMI__CONFIGURATION_PARAMETER__


namespace earth_rover_hmi
{

  //! Container for a configuration parameter.
  /*!
   *  The container stores both the current value of the configuration parameter and the value stored in the VCU and
   *  has a flag indicating whether the value stored in the VCU was already retrieved or not.
   * 
   *  The stored type needs an operator!=.
   * 
   *  \tparam ConfigurationParameter_t Type of the stored configuration parameter.
   * 
   *  \ingroup HMI
   */
  template<typename ConfigurationParameter_t>
  class ConfigurationParameter
  {
    private:

      ConfigurationParameter_t current_value;  //!< Current value of the configuration parameter.
      ConfigurationParameter_t stored_value;   //!< Value of the configuration parameter stored in the VCU.
      bool stored_value_available;             //!< Flag indicating whether the value stored in the VCU is available.
      bool changed;                            //!< Flag indicating whether the configuration parameter changed.

    public:

      //! Constructor.
      /*!
       *  The current value is initialized using the configuration parameter's default constructor.
       */
      ConfigurationParameter()
      :
        current_value {},
        stored_value {},
        stored_value_available {false},
        changed {false}
      {
        ;
      }

      //! Constructor.
      /*!
       *  The current value is initialized using the given value. The stored value, although reported as unavailable,
       *  is initialized using the same value, to prevent errors with types without a default constructor.
       * 
       *  \param default_value Default value for the configuration parameter.
       */
      ConfigurationParameter(ConfigurationParameter_t default_value)
      :
        current_value {default_value},
        stored_value {std::move(default_value)},
        stored_value_available {false},
        changed {false}
      {
        ;
      }

      //! Default destructor.
      ~ConfigurationParameter() = default;

      //! Set the current value.
      /*!
       *  \param new_value New value.
       */
      void setCurrentValue(ConfigurationParameter_t new_value)
      {
        if(new_value != current_value)
        {
          changed = true;
          current_value = new_value;
        }
      }

      //! Check whether the current value changed.
      /*!
       *  Check whether the current value changed since the last getCurrentValue() call with reset_changed = true.
       * 
       *  \return True if the current value changed.
       */
      bool isCurrentValueChanged()
      {
        return changed;
      }

      //! Get the current value.
      /*!
       *  \param reset_changed True to reset the changed flag, false to keep its current state.
       *  \return The current value.
       */
      ConfigurationParameter_t & getCurrentValue(bool reset_changed = false)
      {
        if(reset_changed)
        {
          changed = false;
        }
        return current_value;
      }

      //! Set the value stored on the VCU.
      /*!
       *  \param new_value New value.
       *  \param is_available True to mark the stored configuration parameter as available; false to mark it as
       *                      unavailable (e.g. still incomplete).
       */
      void setStoredValue(ConfigurationParameter_t new_value, bool is_available = true)
      {
        stored_value = new_value;
        stored_value_available = is_available;
      }

      //! Check whether the stored value is available.
      /*!
       *  \return True if the stored value is available, fals if it isn't.
       */
      bool isStoredValueAvailable()
      {
        return stored_value_available;
      }

      //! Get the stored value.
      /*!
       *  \return The stored value.
       */
      const ConfigurationParameter_t & getStoredValue()
      {
        return stored_value;
      }

  };

}

#endif