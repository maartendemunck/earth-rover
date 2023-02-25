// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__SERVO_STATE__
#define __EARTH_ROVER__SERVO_STATE__

#include "configuration-parameter.hpp"
#include "limit-value.hpp"
#include <cstdint>
#include <utility>

namespace earth_rover {

    class ServoConfigParams {
      public:
        uint8_t input_channel;
        uint16_t pulse_width_minimum;
        uint16_t pulse_width_center;
        uint16_t pulse_width_maximum;
        bool enforce_pulse_width_limits;

        ServoConfigParams(uint8_t input_channel, uint16_t pulse_width_minimum,
                          uint16_t pulse_width_center, uint16_t pulse_width_maximum,
                          bool enforce_pulse_width_limits)
            : input_channel{input_channel}, pulse_width_minimum{pulse_width_minimum},
              pulse_width_center{pulse_width_center}, pulse_width_maximum{pulse_width_maximum},
              enforce_pulse_width_limits{enforce_pulse_width_limits} {
            ;
        }

        bool operator!=(const ServoConfigParams &rhs) {
            return (input_channel != rhs.input_channel
                    || pulse_width_minimum != rhs.pulse_width_minimum
                    || pulse_width_center != rhs.pulse_width_center
                    || pulse_width_maximum != rhs.pulse_width_maximum
                    || enforce_pulse_width_limits != rhs.enforce_pulse_width_limits);
        }
    };

    class ServoState : public ConfigParameter<ServoConfigParams> {
      private:
        int16_t position;

      public:
        ServoState(ServoConfigParams default_config)
            : ConfigParameter{std::move(default_config)}, position{0} {
            ;
        }

        void setCurrentPosition(int16_t new_position) {
            position = limit_value<int16_t>(new_position, -1000, 1000);
        }

        int16_t getCurrentPosition() {
            return position;
        }
    };

}  // namespace earth_rover

#endif