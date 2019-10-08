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
        uint8_t pin_number;               //!< I/O pin used to control the servo.
        uint16_t minimum_pulse_width;     //!< Minimum (normalized position = -1000) pulse width.
        uint16_t maximum_pulse_width;     //!< Maximum (normalized position = +1000) pulse width.
        uint16_t center_pulse_width;      //!< Center (normalized position = 0) pulse width.
        uint16_t initial_pulse_width;     //!< Initial pulse width.
        bool enforce_pulse_width_limits;  //!< Enforce the minimum and maximum pulse widths when calling setPulseWidth.
      };

    private:

      static constexpr uint16_t default_minimum_pulse_width {1000U};    //!< Default minimum pulse widths.
      static constexpr uint16_t default_maximum_pulse_width {2000U};    //!< Default maximum pulse width.
      static constexpr uint16_t default_center_pulse_width {1500U};     //!< Default center pulse width.
      static constexpr uint16_t default_initial_pulse_width {1500U};    //!< Default initial pulse width.
      static constexpr bool default_enforce_pulse_width_limits {true};  //!< Enforce pulse widths by default.

      Servo servo;                   //!< Servo instance.
      Configuration configuration;   //!< Current configuration.
      uint16_t current_pulse_width;  //!< Current pulse width.

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
       *  \return The current configuration of the servo.
       */
      Configuration getConfiguration() const;

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