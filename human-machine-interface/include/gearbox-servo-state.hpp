//! Servo state (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_HMI__GEARBOX_SERVO_STATE__
#define __EARTH_ROVER_HMI__GEARBOX_SERVO_STATE__

#include "configuration-parameter.hpp"
#include "limit-value.hpp"
#include <cstdint>

namespace earth_rover_hmi {

    //! Gearbox servo configuration.
    /*!
     *  \tparam reverse_gears Number of reverse gears (>= 0).
     *  \tparam has_neutral_gear True if the gearbox has a neutral gear, false if it doesn't.
     *  \tparam forward_gears Number of forward gears (>= 1).
     *
     *  \ingroup HMI
     */
    template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
    class GearboxServoConfigParams {
      public:
        //! Input channel.
        uint8_t input_channel;

      private:
        //! Total number of gears.
        static constexpr uint8_t total_gear_count{reverse_gears + (has_neutral_gear ? 1 : 0)
                                                  + forward_gears};
        //! Pulse widths for all gears.
        uint16_t pulse_widths[total_gear_count];

      public:
        //! Constructor
        /*!
         *  \param input_channel Input channel
         *  \param pulse_widths Array with the pulse widths for all gears (from low to high)
         */
        GearboxServoConfigParams(uint8_t input_channel,
                                 const uint16_t pulse_widths[total_gear_count])
            : input_channel{input_channel} {
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
        bool operator!=(const GearboxServoConfigParams &rhs) const {
            for(unsigned gear = 0; gear < total_gear_count; ++gear) {
                if(pulse_widths[gear] != rhs.pulse_widths[gear]) {
                    return true;
                }
            }
            return input_channel != rhs.input_channel;
        }

        //! Set pulse widths for all gears.
        /*!
         *  \param new_pulse_widths Array with the pulse widths for all gears (from low to high)
         */
        void setPulseWidths(const uint16_t new_pulse_widths[total_gear_count]) {
            for(unsigned gear = 0; gear < total_gear_count; ++gear) {
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
        void setPulseWidth(int8_t gear, uint16_t pulse_width) {
            if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0)) {
                if(gear > 0 && !has_neutral_gear) {
                    --gear;
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
        uint16_t getPulseWidth(uint8_t gear) {
            if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0)) {
                if(gear > 0 && !has_neutral_gear) {
                    --gear;
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
        : public ConfigurationParameter<
              GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>> {
      private:
        //! Alias for the GearboxServoConfigParams template type.
        using GearboxServoConfigParams_t
            = GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>;

        //! Current gear.
        int8_t current_gear{has_neutral_gear ? 0 : 1};

      public:
        //! Constructor.
        /*!
         *  \param default_configuration Default configuration.
         */
        GearboxServoState(GearboxServoConfigParams_t default_configuration)
            : ConfigurationParameter<GearboxServoConfigParams_t>{std::move(default_configuration)} {
            static_assert(forward_gears > 0, "At least one forward gear is required");
        }

        //! Default destructor.
        ~GearboxServoState() = default;

        //! Set the current gear.
        /*!
         *  Attempts to shift to a non-existing gear are silently ignored.
         *
         *  \param new_gear Gear to shift to.
         */
        void setCurrentGear(int8_t new_gear) {
            if(new_gear >= -reverse_gears && new_gear <= forward_gears
               && (has_neutral_gear || new_gear != 0)) {
                current_gear = new_gear;
            }
            else {
                ;  // Silently ignore non-existing gears.
            }
        }

        //! Get the current gear.
        /*!
         *  \return The current gear.
         */
        int8_t getCurrentGear() { return current_gear; }

        //! Shift up one gear.
        void shiftUp() {
            current_gear = limit_value(int8_t(current_gear + 1), int8_t(-reverse_gears),
                                       int8_t(forward_gears));
            if(current_gear == 0 && !has_neutral_gear) {
                current_gear = 1;
            }
        }

        //! Shift down one gear.
        void shiftDown() {
            current_gear = limit_value(int8_t(current_gear - 1), int8_t(-reverse_gears),
                                       int8_t(forward_gears));
            if(current_gear == 0 && !has_neutral_gear) {
                if(reverse_gears >= 1) {
                    current_gear = -1;
                }
                else {
                    current_gear = 1;
                }
            }
        }
    };

}  // namespace earth_rover_hmi

#endif