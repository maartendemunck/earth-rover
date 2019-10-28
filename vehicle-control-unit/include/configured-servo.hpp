//! Generic servo device driver (interface).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__CONFIGURED_SERVO__
#define __EARTH_ROVER_VCU__CONFIGURED_SERVO__


#include <Servo.h>


namespace earth_rover_vcu
{

  //! Generic servo device driver.
  /*!
   *  \ingroup VCU
   */
  class ConfiguredServo
  {

    public:

      //! Servo configuration.
      /*!
       *  \ingroup VCU
       */
      struct Configuration {
        uint16_t minimum_pulse_width;     //!< Minimum (normalized position = -1000) pulse width.
        uint16_t center_pulse_width;      //!< Center (normalized position = 0) pulse width.
        uint16_t maximum_pulse_width;     //!< Maximum (normalized position = +1000) pulse width.
        uint16_t initial_pulse_width;     //!< Initial pulse width.
        bool enforce_pulse_width_limits;  //!< Enforce the minimum and maximum pulse widths when calling setPulseWidth.

        //! Constructor.
        /*!
         *  \param minimum_pulse_width Minimum (normalized position = -1000) pulse width.
         *  \param center_pulse_width Center (normalized position = 0) pulse width.
         *  \param maximum_pulse_width Maximum (normalized position = +1000) pulse width.
         *  \param initial_pulse_width Initial pulse width.
         *  \param enforce_pulse_width_limits Enforce the minimum and maximum pulse widths when calling setPulseWidth.
         */
        Configuration(
          uint16_t minimum_pulse_width, uint16_t center_pulse_width, uint16_t maximum_pulse_width,
          uint16_t initial_pulse_width, bool enforce_pulse_width_limits)
        :
          minimum_pulse_width {minimum_pulse_width},
          center_pulse_width {center_pulse_width},
          maximum_pulse_width {maximum_pulse_width},
          initial_pulse_width {initial_pulse_width},
          enforce_pulse_width_limits {enforce_pulse_width_limits}
        {
          ;
        }

        //! != operator.
        /*!
         *  \param rhs Right hand side of the comparison.
         *  \return True if the configurations are different, false if they are equal.
         */
        bool operator!= (const Configuration & rhs) const
        {
          return minimum_pulse_width != rhs.minimum_pulse_width
                 || center_pulse_width != rhs.center_pulse_width
                 || maximum_pulse_width != rhs.maximum_pulse_width
                 || initial_pulse_width != rhs.initial_pulse_width
                 || enforce_pulse_width_limits != rhs.enforce_pulse_width_limits;
        }
      };

    private:

      Configuration default_configuration;  //!< Default configuration.

      const uint8_t pin_number;             //!< I/O pin used to control the servo.
      Servo servo;                          //!< Servo instance.
      Configuration configuration;          //!< Current configuration.
      uint16_t current_pulse_width;         //!< Current pulse width.
      bool changed;                         //!< True if the configuration is changed.

    public:

      //! Constructor.
      /*!
       *  \param pin_number I/O pin used to control the servo.
       */
      ConfiguredServo(uint8_t pin_number);

      //! Constructor.
      /*!
       *  \param pin_number I/O pin used to control the servo.
       *  \param minimum_pulse_width Pulse width of the (normalized) minimum position.
       *  \param maximum_pulse_width Pulse width of the (normalized) maximum position.
       *  \param center_pulse_width Pulze width of the (normalized) center position.
       *  \param enforce_pulse_width_limits True to enforce the pulse width limits in setPulseWidth calls.
       */
      ConfiguredServo(uint8_t pin_number, uint16_t minimum_pulse_width, uint16_t maximum_pulse_width,
                      uint16_t center_pulse_width, bool enforce_pulse_width_limits);

      //! Destructor.
      ~ConfiguredServo();

      //! Initialize the servo using the default configuration or the configuration specified in the constructor.
      void setup();

      //! Initialize the servo using a given configuration.
      /*!
       *  \param new_configuration New configuration for the servo.
       */
      void setup(const Configuration & new_configuration);

      //! Set the servo's normalized position.
      /*!
       *  \param position Normalized position (-1000...0...+1000).
       */
      void setPosition(int16_t position);

      //! Set the servo's pulse width.
      /*!
       *  \param pulse_width Pulse width. If enforcing pulse widths, this pulse width is limited to the allowed interval.
       */
      void setPulseWidth(uint16_t pulse_width);

      //! Restore the default configuration.
      void setDefaultConfiguration();

      //! Set the configuration parameters of the servo.
      /*!
       *  Changing the pin number is not allowed.
       * 
       *  \param new_configuration New configuration for the servo. 
       */
      void setConfiguration(const Configuration & new_configuration);

      //! Get the configuration parameters of the servo.
      /*!
       *  \return A const reference to the current configuration of the servo.
       */
      const Configuration & getConfiguration() const
      {
        return configuration;
      }

      bool isConfigurationChanged()
      {
        return changed;
      }

      void resetConfigurationChanged()
      {
        changed = false;
      }

    private:

      //! Limit the pulse width if enforcing pulse width limits.
      /*!
       *  Limit the pulse width to the allowed interval if pulse width limits are enforced.
       * 
       *  \param requested_pulse_width Requested pulse width.
       *  \return Corrected pulse width if enforcing pulse width limits; requested pulse width if not.
       */
      uint16_t correctPulseWidth(uint16_t requested_pulse_width) const;

  };

}


#endif