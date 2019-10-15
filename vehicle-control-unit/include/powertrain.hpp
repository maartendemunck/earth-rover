//! Powertrain device driver for the Earth Rover's VCU (interface and inline part of the implementation).
/*!
 *  Device driver for the Earth Rover's powertrain, controlling the electronic speed controller (ESC) and the gearbox
 *  servo.
 * 
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__POWERTRAIN__
#define __EARTH_ROVER_VCU__POWERTRAIN__


#include <Arduino.h>
#include <cstdint>
#include "configured-servo.hpp"


namespace earth_rover_vcu
{

  //! Powertrain (ESC and gearbox servo) device driver for the Earth Rover's VCU.
  /*!
   *  This class abstracts the powertrain used in the Earth Rover.
   * 
   *  \ingroup VCU
   */
  class Powertrain
  {
    private:

      uint8_t esc_pin_number;            //!< I/O pin used to control the ESC.
      uint8_t gearbox_servo_pin_number;  //!< I/O pin used to control the gearbox servo.
      ConfiguredServo esc;               //!< ESC controller.
      ConfiguredServo gearbox_servo;     //!< Gearbox servo controller.
      uint16_t gearbox_pulse_widths[3];  //!< Gearbox servo pulse widths for the different gears.
      int16_t current_throttle_setting;  //!< Current (normalized) throttle setting (-1000...0...+1000).
      int8_t current_gear;               //!< Current gear.
      int8_t requested_gear;             //!< Requested gear.
      bool is_driving;                   //!< True if we're driving, false otherwise.
      elapsedMillis since_driving;       //!< Elapsed time since we started driving.

    public:

      //! Constructor.
      /*!
       *  \param esc_pin_number Output pin used to control the ESC.
       *  \param gearbox_servo_pin_number Output pin used to control the gearbox servo.
       */
      Powertrain(uint8_t esc_pin_number, uint8_t gearbox_servo_pin_number);

      //! Default destructor.
      ~Powertrain() = default;

      //! Initialize the powertrain.
      void setup();

      //! Spinning loop.
      /*!
       *  \internal
       *  To prevent overloading the gearbox servo, the motor should be running when shifting into first or second gear.
       *  Shift commands are recorded by the setGear function, but the shifting is performed by the spinning loop, when
       *  the motor is running.
       */
      void spinOnce();

      //! Set the (normalized) throttle setting.
      /*!
       *  If driving backwards requires shifting into reverse, this function should handle that.
       * 
       *  \param throttle_setting Normalized (-1000 = full speed backwards, 0 = stop, +1000 = full speed forward).
       */
      void setNormalizedThrottleSetting(int16_t throttle_setting);

      //! Configure the ESC.
      /*!
       *  \param pulse_width_reverse Pulse width for full speed backwards.
       *  \param pulse_width_stop Pulse width for stop.
       *  \param pulse_width_forward Pulse width for full speed forwards.
       */
      void configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop, uint16_t pulse_width_forward);

      //! Set the gear.
      /*!
       *  \param gear Gear to shift to.
       */
      inline void setGear(int8_t gear)
      {
        if(gear >= 0 && gear <= 2)
        {
          requested_gear = gear;
        }
      }

      //! Configure the gearbox servo.
      /*!
       *  \param pulse_width_neutral Pulse width for neutral.
       *  \param pulse_width_low Pulse width for low (first) gear.
       *  \param pulse_width_high Pulse width for high (second) gear.
       */
      void configureGearboxServo(uint16_t pulse_width_neutral, uint16_t pulse_width_low, uint16_t pulse_width_high);

      //! The (minimal) size of the configuration block.
      static constexpr uint16_t configuration_size = 13u;

      //! Save the configuration to a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is written.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is written; false if not (if the buffer isn't large enough).
       */
      bool saveConfiguration(uint8_t * data, uint16_t size);

      //! Load the configuration from a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is read or applied.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is applied; false if not (buffer not large enough or checksum wrong).
       */
      bool loadConfiguration(uint8_t * data, uint16_t size);
      
  };

}

#endif