//! Servo state (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__GEARBOX_SERVO_STATE__
#define __EARTH_ROVER_HMI__GEARBOX_SERVO_STATE__


#include <cstdint>
#include "configuration-parameter.hpp"
#include "limit-value.hpp"


namespace earth_rover_hmi
{

  //! Gearbox servo configuration.
  /*!
   *  \tparam reverse_gears Number of reverse gears (>= 0).
   *  \tparam has_neutral_gear True if the gearbox has a neutral gear, false if it doesn't.
   *  \tparam forward_gears Number of forward gears (>= 1).
   *
   *  \ingroup HMI
   */
  template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
  class GearboxServoConfigParams
  {
    public:

      //! Input channel.
      uint8_t input_channel;

    private:
      //! Total number of gears.
      static constexpr uint8_t total_gear_count {reverse_gears + (has_neutral_gear? 1: 0) + forward_gears};
      //! Pulse widths for all gears.
      uint16_t pulse_widths[total_gear_count];

    public:

      //! Constructor
      /*!
       *  \param input_channel Input channel
       *  \param pulse_widths Array with the pulse widths for all gears (from low to high)
       */
      GearboxServoConfigParams(uint8_t input_channel, const uint16_t pulse_widths[total_gear_count])
      :
        input_channel {input_channel}
      {
        static_assert(reverse_gears <= 127, "no more than 127 reverse gears supported");
        static_assert(forward_gears <= 127, "no more than 127 forward gears supported");
        setPulseWidths(pulse_widths);
      }

      //! Default destructor.
      ~GearboxServoConfigParams() = default;

      //! Compare two gearbox servo configurations.
      /*!
       *  \param rhs Right hand side operand.
       *  \return True if both configurations are different, false if they are equal.
       */
      bool operator!= (const GearboxServoConfigParams & rhs) const
      {
        for(unsigned gear = 0; gear < total_gear_count; ++ gear)
        {
          if(pulse_widths[gear] != rhs.pulse_widths[gear])
          {
            return true;
          }
        }
        return false;
      }

      //! Set pulse widths for all gears.
      /*!
       *  \param new_pulse_widths Array with the pulse widths for all gears (from low to high)
       */
      void setPulseWidths(const uint16_t new_pulse_widths[total_gear_count])
      {
        for(unsigned gear = 0; gear < total_gear_count; ++ gear)
        {
          pulse_widths[gear] = new_pulse_widths[gear];
        }
      }

      //! Set the pulse width for a single gear.
      /*!
       *  Attempts to configure non-existing gears are silently ignored.
       * 
       *  \param gear Gear (negative for reverse gears, positive for forward gears).
       *  \param pulse_width Pulse width (in µs) for that gear.
       */
      void setPulseWidth(int8_t gear, uint16_t pulse_width)
      {
        if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0))
        {
          if(gear > 0 && !has_neutral_gear)
          {
            -- gear;
          }
          pulse_widths[gear + reverse_gears] = pulse_width;
        }
        else  // Invalid gear.
        {
          ;
        }
      }

      //! Set the pulse width for a single gear.
      /*!
       *  \param gear Gear (negative for reverse gears, positive for forward gears).
       *  \return Pulse width (in µs) for that gear of 0 if the gear doesn't exist.
       */
      uint16_t getPulseWidth(uint8_t gear)
      {
        if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0))
        {
          if(gear > 0 && !has_neutral_gear)
          {
            -- gear;
          }
          return pulse_widths[gear + reverse_gears];
        }
        else  // Invalid gear.
        {
          return 0;
        }
      }
  };


  //! Store the configuration and state of a gearbox servo.
  /*!
   *  \tparam reverse_gears Number of reverse gears (>= 0).
   *  \tparam has_neutral_gear True if the gearbox has a neutral gear, false if it doesn't.
   *  \tparam forward_gears Number of forward gears (>= 1).
   * 
   *  \ingroup HMI
   */
  template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
  class GearboxServoState
  {
    private:

      //! Alias for the configuration type.
      using Configuration_t = GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>;

      ConfigurationParameter<Configuration_t> configuration;  //!< Gearbox servo configuration.
      int8_t gear;                                            //!< Current gear.

    public:

      //! Constructor.
      /*!
       *  \param default_configuration Default configuration.
       */
      GearboxServoState(Configuration_t default_configuration)
      :
        configuration {std::move(default_configuration)},
        gear {has_neutral_gear? 0: 1}
      {
        static_assert(forward_gears > 0, "At least one forward gear is required");
        static_assert(forward_gears <= 15, "Maximum 15 forward gears are supported");
        static_assert(reverse_gears <= 7, "Maximum 7 reverse gears are supported");
      }

      //! Default destructor.
      ~GearboxServoState() = default;

      //! Set the new configuration.
      /*!
       *  \param new_configuration New configuration.
       */
      void setCurrentConfiguration(const Configuration_t & new_configuration)
      {
        return configuration.setCurrentValue(new_configuration);
      }
      
      //! Check whether the current configuration is changed.
      /*!
       *  Wheck whether the current configuration is changed since the last getCurrentConfiguration() call with
       *  reset_changed = true.
       * 
       *  \return True if the current configuration is changed, false if not.
       */
      bool isCurrentConfigurationChanged()
      {
        return configuration.isCurrentValueChanged();
      }

      //! Get the current configuration.
      /*!
       *  \param reset_changed True to reset the changed flag, false to keep its current state.
       *  \return A const reference to the current configuration.
       */
      const Configuration_t & getCurrentConfiguration(bool reset_changed = true)
      {
        return configuration.getCurrentValue(reset_changed);
      }

      //! Set the current gear.
      /*!
       *  Attempts to shift to a non-existing gear are silently ignored.
       * 
       *  \param new_gear Gear to shift to.
       */
      void setCurrentGear(int8_t new_gear)
      {
        if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0))
        {
          gear = new_gear;
        }
        else
        {
          ;  // Silently ignore non-existing gears.
        }
      }

      //! Get the current gear.
      /*!
       *  \return The current gear.
       */
      int8_t getCurrentGear()
      {
        return gear;
      }

      //! Shift up one gear.
      void shiftUp()
      {
        gear = limit_value(int8_t(gear + 1), int8_t(-reverse_gears), int8_t(forward_gears));
        if(gear == 0 && !has_neutral_gear)
        {
          gear = 1;
        }
      }

      //! Shift down one gear.
      void shiftDown()
      {
        gear = limit_value(int8_t(gear - 1), int8_t(-reverse_gears), int8_t(forward_gears));
        if(gear == 0 && !has_neutral_gear)
        {
          if(reverse_gears >= 1)
          {
            gear = -1;
          }
          else
          {
            gear = 1;
          }
        }
      }
      
  };

}

#endif