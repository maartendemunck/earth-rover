// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "configured-servo.hpp"
#include <algorithm>

namespace earth_rover {

    ConfiguredServo::ConfiguredServo(uint8_t pin_number)
        : default_config{1000u, 1500u, 2000u, 1500u, true}, pin_number{pin_number}, servo{},
          config{default_config}, current_pulse_width{config.initial_pulse_width} {
        ;
    }

    ConfiguredServo::ConfiguredServo(uint8_t pin_number, uint16_t minimum_pulse_width,
                                     uint16_t center_pulse_width, uint16_t maximum_pulse_width,
                                     bool enforce_pulse_width_limits)
        : default_config{minimum_pulse_width, center_pulse_width, maximum_pulse_width,
                         center_pulse_width, enforce_pulse_width_limits},
          pin_number{pin_number}, servo{}, config{default_config}, current_pulse_width{
                                                                       config.initial_pulse_width} {
        ;
    }

    ConfiguredServo::~ConfiguredServo() {
        servo.writeMicroseconds(config.initial_pulse_width);
        servo.detach();
    }

    void ConfiguredServo::setup() {
        servo.attach(pin_number);
        setPulseWidth(current_pulse_width);
    }

    void ConfiguredServo::setup(const Config &new_config) {
        config = new_config;
        setPulseWidth(current_pulse_width);
    }

    void ConfiguredServo::setPosition(int16_t position) {
        if(position <= -1000) {
            current_pulse_width = config.minimum_pulse_width;
        }
        else if(position == 0) {
            current_pulse_width = config.center_pulse_width;
        }
        else if(position >= 1000) {
            current_pulse_width = config.maximum_pulse_width;
        }
        else if(position < 0) {
            current_pulse_width = config.center_pulse_width
                                  + int16_t(config.center_pulse_width - config.minimum_pulse_width)
                                        * position / 1000;
        }
        else  // position > 0)
        {
            current_pulse_width = config.center_pulse_width
                                  + int16_t(config.maximum_pulse_width - config.center_pulse_width)
                                        * position / 1000;
        }
        servo.writeMicroseconds(current_pulse_width);
    }

    void ConfiguredServo::setPulseWidth(uint16_t pulse_width) {
        current_pulse_width = correctPulseWidth(pulse_width);
        servo.writeMicroseconds(current_pulse_width);
    }

    void ConfiguredServo::setDefaultConfig() {
        if(config != default_config) {
            config = default_config;
            config_is_changed = true;
            setPulseWidth(current_pulse_width);
        }
    }

    void ConfiguredServo::setConfig(const ConfiguredServo::Config &new_config) {
        if(new_config != config) {
            config = new_config;
            config_is_changed = true;
            if(config.enforce_pulse_width_limits) {
                setPulseWidth(current_pulse_width);
            }
        }
    }

    const ConfiguredServo::Config &ConfiguredServo::getConfig() const {
        return config;
    }
    bool ConfiguredServo::isConfigChanged() {
        return config_is_changed;
    }

    void ConfiguredServo::markConfigSaved() {
        config_is_changed = false;
    }

    uint16_t ConfiguredServo::correctPulseWidth(uint16_t requested_pulse_width) const {
        if(config.enforce_pulse_width_limits) {
            // The interval could be reversed, or even V-shaped (if both positive and negative
            // normalized positions move the servo in the same direction from the center position).
            const auto minimum_pulse_width
                = std::min({config.minimum_pulse_width, config.center_pulse_width,
                            config.maximum_pulse_width});
            const auto maximum_pulse_width
                = std::max({config.minimum_pulse_width, config.center_pulse_width,
                            config.maximum_pulse_width});
            return std::max(minimum_pulse_width,
                            std::min(maximum_pulse_width, requested_pulse_width));
        }
        else {
            return requested_pulse_width;
        }
    }

}  // namespace earth_rover