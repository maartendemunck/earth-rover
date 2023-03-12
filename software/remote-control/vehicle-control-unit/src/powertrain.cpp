// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "powertrain.hpp"

namespace earth_rover {

    Powertrain::Powertrain(uint8_t esc_pin_number, uint8_t gearbox_servo_pin_number)
        : esc_pin_number{esc_pin_number}, esc{esc_pin_number, 1000u, 1500u, 2000u, true},
          gearbox_servo_pin_number{gearbox_servo_pin_number},
          gearbox_servo{gearbox_servo_pin_number, 1000u, 1500u, 2000u, false},
          gearbox_pulse_widths{1480u, 1150u, 1800u}, current_throttle_setting{0}, current_gear{0} {
        ;
    }

    void Powertrain::setup() {
        current_throttle_setting = 0;
        esc.setup();
        esc.setPosition(current_throttle_setting);
        current_gear = 0;
        gearbox_servo.setup();
        gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
    }

    void Powertrain::spinOnce() {
        if(current_gear != requested_gear) {
            if(requested_gear == 0 || (is_driving && time_since_driving > 200u)) {
                gearbox_servo.setPulseWidth(gearbox_pulse_widths[requested_gear]);
                current_gear = requested_gear;
            }
        }
    }

    void Powertrain::setNormalizedThrottleSetting(int16_t throttle_setting) {
        esc.setPosition(throttle_setting);
        current_throttle_setting = throttle_setting;
        if(!is_driving && abs(current_throttle_setting) >= 150) {
            is_driving = true;
            time_since_driving = 0;
        }
        else if(is_driving && abs(current_throttle_setting) <= 100) {
            is_driving = false;
        }
    }

    void Powertrain::setGear(int8_t gear) {
        if(gear >= 0 && gear <= 2) {
            requested_gear = gear;
        }
    }

    void Powertrain::configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop,
                                  uint16_t pulse_width_forward) {
        ConfiguredServo::Config config{pulse_width_reverse, pulse_width_stop, pulse_width_forward,
                                       pulse_width_stop, true};
        esc.setConfig(config);
        esc.setPosition(current_throttle_setting);
        markChanged();
    }

    void Powertrain::configureGearboxServo(uint16_t pulse_width_neutral, uint16_t pulse_width_low,
                                           uint16_t pulse_width_high) {
        gearbox_pulse_widths[0] = pulse_width_neutral;
        gearbox_pulse_widths[1] = pulse_width_low;
        gearbox_pulse_widths[2] = pulse_width_high;
        gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
        markChanged();
    }

    void Powertrain::configureGearboxServo(int8_t gear, uint16_t pulse_width) {
        if(gear >= 0 && gear <= 2) {
            gearbox_pulse_widths[gear] = pulse_width;
            if(gear == current_gear) {
                gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
                markChanged();
            }
        }
    }

    Powertrain::Config Powertrain::getConfig() {
        Config config;
        auto esc_config = esc.getConfig();
        config.esc.pulse_width_reverse = esc_config.minimum_pulse_width;
        config.esc.pulse_width_stop = esc_config.center_pulse_width;
        config.esc.pulse_width_forward = esc_config.maximum_pulse_width;
        config.gearbox.pulse_width_neutral = gearbox_pulse_widths[0];
        config.gearbox.pulse_width_low = gearbox_pulse_widths[1];
        config.gearbox.pulse_width_high = gearbox_pulse_widths[2];
        return config;
    }

    Powertrain::SerializationResult Powertrain::serialize(uint8_t *buffer, uint16_t size) {
        if(size >= 12u) {
            auto esc_config = esc.getConfig();
            buffer[0] = esc_config.minimum_pulse_width & 0x00ff;
            buffer[1] = (esc_config.minimum_pulse_width & 0xff00) >> 8;
            buffer[2] = esc_config.center_pulse_width & 0x00ff;
            buffer[3] = (esc_config.center_pulse_width & 0xff00) >> 8;
            buffer[4] = esc_config.maximum_pulse_width & 0x00ff;
            buffer[5] = (esc_config.maximum_pulse_width & 0xff00) >> 8;
            buffer[6] = gearbox_pulse_widths[0] & 0x00ff;
            buffer[7] = (gearbox_pulse_widths[0] & 0xff00) >> 8;
            buffer[8] = gearbox_pulse_widths[1] & 0x00ff;
            buffer[9] = (gearbox_pulse_widths[1] & 0xff00) >> 8;
            buffer[10] = gearbox_pulse_widths[2] & 0x00ff;
            buffer[11] = (gearbox_pulse_widths[2] & 0xff00) >> 8;
            for(unsigned index = 12; index < size; ++index) {
                buffer[index] = 0xff;
            }
            return SerializationResult::SAVE_IN_NEW_RECORD;
        }
        else {
            return SerializationResult::ERROR;
        }
    }

    bool Powertrain::deserialize(uint8_t *buffer, uint16_t size) {
        if(size >= 12u) {
            auto pulse_width_reverse = uint16_t(buffer[0] | (buffer[1] << 8));
            auto pulse_width_stop = uint16_t(buffer[2] | (buffer[3] << 8));
            auto pulse_width_forward = uint16_t(buffer[4] | (buffer[5] << 8));
            decltype(esc)::Config config{pulse_width_reverse, pulse_width_stop, pulse_width_forward,
                                         pulse_width_stop, true};
            esc.setConfig(config);
            esc.setPosition(current_throttle_setting);
            gearbox_pulse_widths[0] = uint16_t(buffer[6] | (buffer[7] << 8));
            gearbox_pulse_widths[1] = uint16_t(buffer[8] | (buffer[9] << 8));
            gearbox_pulse_widths[2] = uint16_t(buffer[10] | (buffer[11] << 8));
            return false;
        }
        else {
            return true;
        }
    }

}  // namespace earth_rover