// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__GEARBOX_SERVO_STATE__
#define __EARTH_ROVER__GEARBOX_SERVO_STATE__

#include "configuration-parameter.hpp"
#include "limit-value.hpp"
#include <Arduino.h>
#include <cstdint>
#include <utility>

namespace earth_rover {

    template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
    class GearboxServoConfigParams {

      private:
        static constexpr uint8_t total_gears{reverse_gears + (has_neutral_gear ? 1 : 0)
                                             + forward_gears};
        uint16_t pulse_widths[total_gears];

      public:
        GearboxServoConfigParams(const uint16_t pulse_widths[total_gears]) {
            static_assert(reverse_gears <= 7, "no more than 7 reverse gears supported");
            static_assert(forward_gears <= 15, "no more than 15 forward gears supported");
            setPulseWidthsForAllGears(pulse_widths);
        }

        bool operator!=(const GearboxServoConfigParams &rhs) const {
            for(unsigned gear = 0; gear < total_gears; ++gear) {
                if(pulse_widths[gear] != rhs.pulse_widths[gear]) {
                    return true;
                }
            }
            return false;
        }

        bool isGearAvailable(int8_t new_gear) {
            return (new_gear >= -reverse_gears && (new_gear != 0 || has_neutral_gear)
                    && new_gear <= forward_gears);
        }

        void setPulseWidthsForAllGears(const uint16_t new_pulse_widths[total_gears]) {
            for(unsigned gear = 0; gear < total_gears; ++gear) {
                pulse_widths[gear] = new_pulse_widths[gear];
            }
        }

        void setPulseWidthForSingleGear(int8_t gear, uint16_t new_pulse_width) {
            if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0)) {
                if(gear > 0 && !has_neutral_gear) {
                    --gear;
                }
                pulse_widths[gear + reverse_gears] = new_pulse_width;
            }
            else {
                ;
            }
        }

        uint16_t getPulseWidthForGear(int8_t gear) {
            if(gear >= -reverse_gears && gear <= forward_gears && (has_neutral_gear || gear != 0)) {
                if(gear > 0 && !has_neutral_gear) {
                    --gear;
                }
                return pulse_widths[gear + reverse_gears];
            }
            else {
                return 0;
            }
        }
    };

    using TwoSpeedGearboxServoConfigParams = GearboxServoConfigParams<0, false, 2>;

    template <uint8_t reverse_gears, bool has_neutral_gear, uint8_t forward_gears>
    class GearboxServoState
        : public ConfigParameter<
              GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>> {
      private:
        using GearboxServoConfigParams_t
            = GearboxServoConfigParams<reverse_gears, has_neutral_gear, forward_gears>;

        int8_t current_gear{has_neutral_gear ? 0 : 1};

      public:
        GearboxServoState(GearboxServoConfigParams_t default_config)
            : ConfigParameter<GearboxServoConfigParams_t>{std::move(default_config)} {
            static_assert(forward_gears > 0, "At least one forward gear is required");
        }

        void setCurrentGear(int8_t new_gear) {
            if(new_gear >= -reverse_gears && new_gear <= forward_gears
               && (has_neutral_gear || new_gear != 0)) {
                current_gear = new_gear;
            }
            else {
                ;  // Silently ignore non-existing gears.
            }
        }

        int8_t getCurrentGear() {
            return current_gear;
        }

        void shiftUpOneGear() {
            current_gear = limit_value(int8_t(current_gear + 1), int8_t(-reverse_gears),
                                       int8_t(forward_gears));
            if(current_gear == 0 && !has_neutral_gear) {
                current_gear = 1;
            }
        }

        void shiftDownOneGear() {
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

    using TwoSpeedGearboxServoState = GearboxServoState<0, false, 2>;

}  // namespace earth_rover

#endif