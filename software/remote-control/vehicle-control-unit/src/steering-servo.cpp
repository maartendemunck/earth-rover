// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "steering-servo.hpp"

namespace earth_rover {

    SteeringServo::SteeringServo(uint8_t pin_number)
        : pin_number{pin_number}, steering_servo{pin_number, 2000u, 1500u, 1000u, true},
          current_steering_angle{0} {
        ;
    }

    void SteeringServo::setup() {
        current_steering_angle = 0;
        steering_servo.setup();
        steering_servo.setPosition(current_steering_angle);
    }

    void SteeringServo::spinOnce() {
        ;
    }

    void SteeringServo::setNormalizedSteeringAngle(int16_t steering_angle) {
        steering_servo.setPosition(steering_angle);
        current_steering_angle = steering_angle;
    }

    void SteeringServo::configureSteeringServo(uint16_t pulse_width_left,
                                               uint16_t pulse_width_center,
                                               uint16_t pulse_width_right) {
        decltype(steering_servo)::Config config{pulse_width_left, pulse_width_center,
                                                pulse_width_right, pulse_width_center, true};
        steering_servo.setConfig(config);
        steering_servo.setPosition(current_steering_angle);
        markChanged();
    }

    SteeringServo::Config SteeringServo::getConfig() {
        auto config = steering_servo.getConfig();
        return Config{config.minimum_pulse_width, config.center_pulse_width,
                      config.maximum_pulse_width};
    }

    SteeringServo::SerializationResult SteeringServo::serialize(uint8_t *data, uint16_t size) {
        if(size >= 6u) {
            auto config = steering_servo.getConfig();
            data[0] = config.minimum_pulse_width & 0x00ff;
            data[1] = (config.minimum_pulse_width & 0xff00) >> 8;
            data[2] = config.center_pulse_width & 0x00ff;
            data[3] = (config.center_pulse_width & 0xff00) >> 8;
            data[4] = config.maximum_pulse_width & 0x00ff;
            data[5] = (config.maximum_pulse_width & 0xff00) >> 8;
            for(uint16_t index = 6; index < size; ++index) {
                data[index] = 0xff;
            }
            return SerializationResult::SAVE_IN_NEW_RECORD;
        }
        else {
            return SerializationResult::ERROR;
        }
    }

    bool SteeringServo::deserialize(uint8_t *data, uint16_t size) {
        if(size >= 6u) {
            auto pulse_width_left = uint16_t(data[0] | (data[1] << 8));
            auto pulse_width_center = uint16_t(data[2] | (data[3] << 8));
            auto pulse_width_right = uint16_t(data[4] | (data[5] << 8));
            decltype(steering_servo)::Config config{pulse_width_left, pulse_width_center,
                                                    pulse_width_right, pulse_width_center, true};
            steering_servo.setConfig(config);
            steering_servo.setPosition(current_steering_angle);
            return false;
        }
        else {
            return true;
        }
    }

}  // namespace earth_rover