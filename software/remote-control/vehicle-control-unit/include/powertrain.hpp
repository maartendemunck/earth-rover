//! Powertrain device driver for the Earth Rover's VCU (interface and inline part of the
//! implementation).
/*!
 *  Device driver for the Earth Rover's powertrain, controlling the electronic speed controller
 * (ESC) and the gearbox servo.
 *
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_VCU__POWERTRAIN__
#define __EARTH_ROVER_VCU__POWERTRAIN__

#include "configured-servo.hpp"
#include <Arduino.h>
#include <cstdint>

namespace earth_rover_vcu {

    //! Powertrain (ESC and gearbox servo) device driver for the Earth Rover's VCU.
    /*!
     *  This class abstracts the powertrain (ESC and two-speed transmission) used in the Earth
     * Rover.
     *
     *  \ingroup VCU
     */
    class Powertrain {
      public:
        //! Powertrain configuration.
        struct Configuration {
            //! ESC or throttle servo configuration.
            struct {
                uint16_t pulse_width_reverse;  //!< Pulse width for full speed backwards.
                uint16_t pulse_width_stop;     //!< Pulse width to stop.
                uint16_t pulse_width_forward;  //!< Pulse width for full speed forward.
            } esc;
            //! Gearbox servo configuration.
            struct {
                uint16_t pulse_width_neutral;  //!< Pulse width for neutral gear.
                uint16_t pulse_width_low;      //!< Pulse width for first (low) gear.
                uint16_t pulse_width_high;     //!< Pulse width for second (high) gear.
            } gearbox;
        };

      private:
        uint8_t esc_pin_number;            //!< I/O pin used to control the ESC.
        uint8_t gearbox_servo_pin_number;  //!< I/O pin used to control the gearbox servo.
        ConfiguredServo esc;               //!< ESC controller.
        ConfiguredServo gearbox_servo;     //!< Gearbox servo controller.
        uint16_t gearbox_pulse_widths[3];  //!< Gearbox servo pulse widths for the different gears.
        uint8_t throttle_input_channel;    //!< Throttle input channel.
        uint8_t gearbox_input_channel;     //!< Gearbox input channel.
        bool input_channel_changed;        //!< True if one of the input channels changed.
        bool save_required;                //!< True if the configuration should be saved to EEPROM.
        int16_t current_throttle_setting;  //!< Current (normalized) throttle setting
                                           //!< (-1000...0...+1000).
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
         *  To prevent overloading the gearbox servo, the motor should be running when shifting into
         * first or second gear. Shift commands are recorded by the setGear function, but the
         * shifting is performed by the spinning loop, when the motor is running.
         */
        void spinOnce();

        //! Set the (normalized) throttle setting.
        /*!
         *  If driving backwards requires shifting into reverse, this function should handle that.
         *
         *  \param throttle_setting Normalized (-1000 = full speed backwards, 0 = stop, +1000 = full
         * speed forward).
         */
        void setNormalizedThrottleSetting(int16_t throttle_setting);

        //! Configure the ESC.
        /*!
         *  \param pulse_width_reverse Pulse width for full speed backwards.
         *  \param pulse_width_stop Pulse width for stop.
         *  \param pulse_width_forward Pulse width for full speed forwards.
         */
        void configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop,
                          uint16_t pulse_width_forward);

        //! Set the gear.
        /*!
         *  \param gear Gear to shift to.
         */
        inline void setGear(int8_t gear) {
            if(gear >= 0 && gear <= 2) {
                requested_gear = gear;
            }
        }

        //! Configure the gearbox servo.
        /*!
         *  \param pulse_width_neutral Pulse width for neutral.
         *  \param pulse_width_low Pulse width for low (first) gear.
         *  \param pulse_width_high Pulse width for high (second) gear.
         */
        void configureGearboxServo(uint16_t pulse_width_neutral, uint16_t pulse_width_low,
                                   uint16_t pulse_width_high);

        //! Configure the gearbox servo.
        /*!
         *  \param gear Gear.
         *  \param pulse_width Pulse width for this gear.
         */
        void configureGearboxServo(int8_t gear, uint16_t pulse_width);

        //! Get the current configuration.
        /*!
         *  \return The current configuration of the steering servo.
         */
        Configuration getConfiguration();

        //! Set the throttle input channel.
        /*!
         *  \param new_input_channel New throttle input channel.
         */
        void setThrottleInputChannel(uint8_t new_input_channel) {
            if(new_input_channel != throttle_input_channel) {
                throttle_input_channel = new_input_channel;
                input_channel_changed = true;
            }
        }

        //! Get the throttle input channel.
        /*!
         *  \return The throttle input channel.
         */
        uint8_t getThrottleInputChannel() { return throttle_input_channel; }

        //! Set the gearbox servo's input channel.
        /*!
         *  \param new_input_channel New gearbox servo input channel.
         */
        void setGearboxInputChannel(uint8_t new_input_channel) {
            if(new_input_channel != gearbox_input_channel) {
                gearbox_input_channel = new_input_channel;
                input_channel_changed = true;
            }
        }

        //! Get the gearbox servo's input channel.
        /*!
         *  \return The gearbox servo's input channel.
         */
        uint8_t getGearboxInputChannel() { return gearbox_input_channel; }

        //! Save the powertrain's configuration to EEPROM.
        void saveConfiguration() {
            if(esc.isConfigurationChanged() || gearbox_servo.isConfigurationChanged()
               || input_channel_changed) {
                save_required = true;
            }
        }

        //! Check whether the powertrain's configuration should be written to EEPROM.
        /*!
         *  \return True if the powertrain's configuration should be written to EEPROM.
         */
        bool saveRequired() { return save_required; }

        //! Save the configuration to a buffer.
        /*!
         *  \param data Pointer to the buffer.
         *  \param size Size of the buffer.
         *  \return true if the configuration is written; false if not (if the buffer isn't large
         * enough).
         */
        bool serialize(uint8_t *data, uint16_t size);

        //! Load the configuration from a buffer.
        /*!
         *  \param data Pointer to the buffer.
         *  \param size Size of the buffer.
         *  \return true if the configuration is applied; false if not (buffer not large enough or
         * checksum wrong).
         */
        bool deserialize(uint8_t *data, uint16_t size);
    };

}  // namespace earth_rover_vcu

#endif
