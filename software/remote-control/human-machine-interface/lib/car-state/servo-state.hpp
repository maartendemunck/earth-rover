//! Servo state (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_HMI__SERVO_STATE__
#define __EARTH_ROVER_HMI__SERVO_STATE__

#include "configuration-parameter.hpp"
#include "limit-value.hpp"
#include <cstdint>
#include <utility>

namespace earth_rover_hmi {

    //! Servo configuration.
    /*!
     *  \ingroup HMI
     */
    class ServoConfigParams {
      public:
        //! Input channel.
        uint8_t input_channel;
        //! Pulse width for minimum input signal.
        uint16_t pulse_width_minimum;
        //! Pulse width for centered input signal.
        uint16_t pulse_width_center;
        //! Pulse width for maximum input signal.
        uint16_t pulse_width_maximum;
        //! Enforce pulse width limits.
        bool enforce_pulse_width_limits;

        //! Create a servo configuration.
        /*!
         *  \param input_channel Input channel
         *  \param pulse_width_minimum Pulse width for minimum input signal.
         *  \param pulse_width_center Pulse width for centered input signal.
         *  \param pulse_width_maximum Pulse width for maximum input signal.
         *  \param enforce_pulse_width_limits
         */
        ServoConfigParams(uint8_t input_channel, uint16_t pulse_width_minimum,
                          uint16_t pulse_width_center, uint16_t pulse_width_maximum,
                          bool enforce_pulse_width_limits)
            : input_channel{input_channel}, pulse_width_minimum{pulse_width_minimum},
              pulse_width_center{pulse_width_center}, pulse_width_maximum{pulse_width_maximum},
              enforce_pulse_width_limits{enforce_pulse_width_limits} {
            ;
        }

        //! Compare two servo configurations.
        /*!
         *  \param rhs Right hand side operand.
         *  \return True if both configurations are different, false if they are equal.
         */
        bool operator!=(const ServoConfigParams &rhs) {
            return (input_channel != rhs.input_channel
                    || pulse_width_minimum != rhs.pulse_width_minimum
                    || pulse_width_center != rhs.pulse_width_center
                    || pulse_width_maximum != rhs.pulse_width_maximum
                    || enforce_pulse_width_limits != rhs.enforce_pulse_width_limits);
        }
    };

    //! Store the configuration and state of a servo.
    /*!
     *  \ingroup HMI
     */
    class ServoState : public ConfigurationParameter<ServoConfigParams> {
      private:
        //!< Current position.
        int16_t position;

      public:
        //! Constructor.
        /*!
         *  \param default_configuration Default configuration.
         */
        ServoState(ServoConfigParams default_configuration)
            : ConfigurationParameter{std::move(default_configuration)}, position{0} {
            ;
        }

        //! Default destructor.
        ~ServoState() = default;

        //! Store the position of the servo.
        /*!
         *  \param new_position New position of the servo (-1000...1000, automatically clipped).
         */
        void setCurrentPosition(int16_t new_position) {
            position = limit_value<int16_t>(new_position, -1000, 1000);
        }

        //! Get the stored position of the servo.
        /*!
         *  \return The stored position of the servo.
         */
        int16_t getCurrentPosition() { return position; }
    };

}  // namespace earth_rover_hmi

#endif