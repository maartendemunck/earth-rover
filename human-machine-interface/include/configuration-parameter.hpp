//! Container for a configuration parameter (interface and template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_HMI__CONFIGURATION_PARAMETER__
#define __EARTH_ROVER_HMI__CONFIGURATION_PARAMETER__

namespace earth_rover_hmi {

    //! Container for a configuration parameter.
    /*!
     *  The container stores both the current value of the configuration parameter and the value
     * stored in the VCU and has a flag indicating whether the value stored in the VCU was already
     * retrieved or not.
     *
     *  The stored type needs an operator!=.
     *
     *  \tparam ConfigurationParameter_t Type of the stored configuration parameter.
     *
     *  \ingroup HMI
     */
    template <typename ConfigurationParameter_t> class ConfigurationParameter {
      private:
        ConfigurationParameter_t value;  //!< Current value of the configuration parameter.
        bool is_available;  //!< Flag indicating whether the value stored in the VCU is or was
                            //!< available.
        bool is_stored;     //!< Flag indicating whether the current value is stored in the VCU.

      public:
        //! Constructor.
        /*!
         *  The current value is initialized using the configuration parameter's default
         * constructor.
         */
        ConfigurationParameter() : value{}, is_available{false}, is_stored{true} { ; }

        //! Constructor.
        /*!
         *  Initialize the current value using the given value. If the is_initialized flag is false
         * (the default), the parameter is reported as unavailable nevertheless. This feature is
         * needed to prevent errors with configuration parameter types without a default
         * constructor.
         *
         *  \param default_value Default value for the configuration parameter.
         *  \param is_available False (default) to mark the configuration parameter as unavailable,
         * true to mark it as available.
         */
        ConfigurationParameter(ConfigurationParameter_t default_value, bool is_available = false)
            : value{default_value}, is_available{is_available}, is_stored{true} {
            ;
        }

        //! Default destructor.
        ~ConfigurationParameter() = default;

        //! Set the value stored in the VCU.
        /*!
         *  \param new_value New value.
         *  \param is_complete True to mark the stored configuration parameter as available; false
         * to mark it as unavailable because it's still incomplete.
         */
        void setStoredConfiguration(const ConfigurationParameter_t &new_value,
                                    bool is_complete = true) {
            value = new_value;
            is_available = is_complete;
        }

        //! Check whether value stored in the VCU is available.
        /*!
         *  \return True if the stored value is available, false if it isn't.
         */
        bool isConfigurationAvailable() { return is_available; }

        //! Set the current value.
        /*!
         *  \param new_value New value.
         */
        void setCurrentConfiguration(ConfigurationParameter_t new_value) {
            if(new_value != value) {
                is_stored = false;
                value = new_value;
            }
        }

        //! Get the current value.
        /*!
         *  \return The current value.
         */
        const ConfigurationParameter_t &getCurrentConfiguration() { return value; }

        //! Check whether the configuration parameter is available and the current value is stored
        //! in the VCU.
        /*!
         *  \return True if the current value is stored in the VCU, false if not.
         */
        bool isCurrentConfigurationStored() { return is_stored; }

        //! Set the current value as stored.
        void setCurrentConfigurationStored() { is_stored = true; }
    };

}  // namespace earth_rover_hmi

#endif