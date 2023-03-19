// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__CONFIGURED_SERVO__
#define __EARTH_ROVER__CONFIGURED_SERVO__

#include <Servo.h>

namespace earth_rover {

    class ConfiguredServo {

      public:
        struct Config {
            uint16_t minimum_pulse_width;
            uint16_t center_pulse_width;
            uint16_t maximum_pulse_width;
            uint16_t initial_pulse_width;
            bool enforce_pulse_width_limits;

            Config(uint16_t minimum_pulse_width, uint16_t center_pulse_width,
                   uint16_t maximum_pulse_width, uint16_t initial_pulse_width,
                   bool enforce_pulse_width_limits)
                : minimum_pulse_width{minimum_pulse_width}, center_pulse_width{center_pulse_width},
                  maximum_pulse_width{maximum_pulse_width},
                  initial_pulse_width{initial_pulse_width}, enforce_pulse_width_limits{
                                                                enforce_pulse_width_limits} {
                ;
            }

            bool operator!=(const Config &rhs) const {
                return minimum_pulse_width != rhs.minimum_pulse_width
                       || center_pulse_width != rhs.center_pulse_width
                       || maximum_pulse_width != rhs.maximum_pulse_width
                       || initial_pulse_width != rhs.initial_pulse_width
                       || enforce_pulse_width_limits != rhs.enforce_pulse_width_limits;
            }
        };

      private:
        Config default_config;

        const uint8_t pin_number;
        Servo servo;
        Config config;
        bool config_is_changed{false};
        uint16_t current_pulse_width;

      public:
        explicit ConfiguredServo(uint8_t pin_number);
        ConfiguredServo(uint8_t pin_number, uint16_t minimum_pulse_width,
                        uint16_t maximum_pulse_width, uint16_t center_pulse_width,
                        bool enforce_pulse_width_limits);
        ~ConfiguredServo();

        void setup();
        void setup(const Config &new_config);

        void setPosition(int16_t position);
        void setPulseWidth(uint16_t pulse_width);

        void setDefaultConfig();
        void setConfig(const Config &new_config);
        const Config &getConfig() const;
        bool isConfigChanged();
        void markConfigSaved();

      private:
        uint16_t correctPulseWidth(uint16_t requested_pulse_width) const;
    };

}  // namespace earth_rover

#endif