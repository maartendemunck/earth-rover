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

  template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
  class GearboxServoConfigParams
  {
    public:

      //! Input channel.
      uint8_t input_channel;

    private:
      static constexpr uint8_t total_gear_count {reverse_gears + (has_neutral_gear? 1: 0) + forward_gears};
      uint16_t pulse_widths[total_gear_count];

    public:
      GearboxServoConfigParams(
        uint8_t input_channel, const uint16_t pulse_widths[total_gear_count])
      :
        input_channel {input_channel}
      {
        setPulseWidths(pulse_widths);
      }

      ~GearboxServoConfigParams() = default;

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

      void setPulseWidths(const uint16_t new_pulse_widths[total_gear_count])
      {
        for(unsigned gear = 0; gear < total_gear_count; ++ gear)
        {
          pulse_widths[gear] = new_pulse_widths[gear];
        }
      }

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


  template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
  class GearboxServoState
  {
    private:

      using Configuration_t = GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>;

      ConfigurationParameter<Configuration_t>
        configuration;  // Gearbox servo configuration.
      int8_t gear;      // Gearbox servo state.

    public:

      GearboxServoState(Configuration_t default_configuration)
      :
        configuration {std::move(default_configuration)},
        gear {has_neutral_gear? 0: 1}
      {
        static_assert(forward_gears > 0, "At least one forward gear is required");
      }

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

      void setCurrentGear(int8_t new_gear)
      {
        gear = new_gear;
      }

      int8_t getCurrentGear()
      {
        return gear;
      }

      void shiftUp()
      {
        gear = limit_value(int8_t(gear + 1), int8_t(-reverse_gears), int8_t(forward_gears));
        if(gear == 0 && !has_neutral_gear)
        {
          gear = 1;
        }
      }

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