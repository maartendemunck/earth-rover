//! Steering servo device driver for the Earth Rover's VCU (implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#include "steering-servo.hpp"

namespace earth_rover_vcu {

    SteeringServo::SteeringServo(uint8_t pin_number)
        : pin_number{pin_number}, steering_servo{pin_number, 2000u, 1500u, 1000u, true},
          input_channel{0u}, input_channel_changed{false}, save_required{false},
          current_steering_angle{0} {
        ;
    }

    void SteeringServo::configureSteeringServo(uint16_t pulse_width_left,
                                               uint16_t pulse_width_center,
                                               uint16_t pulse_width_right) {
        decltype(steering_servo)::Configuration configuration{
            pulse_width_left, pulse_width_center, pulse_width_right, pulse_width_center, true};
        steering_servo.setConfiguration(configuration);
        steering_servo.setPosition(current_steering_angle);
    }

    bool SteeringServo::serialize(uint8_t *data, uint16_t size) {
        if(size >= 7u) {
            auto configuration = steering_servo.getConfiguration();
            data[0] = configuration.minimum_pulse_width & 0x00ff;
            data[1] = (configuration.minimum_pulse_width & 0xff00) >> 8;
            data[2] = configuration.center_pulse_width & 0x00ff;
            data[3] = (configuration.center_pulse_width & 0xff00) >> 8;
            data[4] = configuration.maximum_pulse_width & 0x00ff;
            data[5] = (configuration.maximum_pulse_width & 0xff00) >> 8;
            data[6] = input_channel;
            for(unsigned index = 7; index < size; ++index) {
                data[index] = 0xff;
            }
            // Reset changed and save flags (we write this configuration to EEPROM, so changes are
            // stored).
            steering_servo.resetConfigurationChanged();
            input_channel_changed = false;
            save_required = false;
            return true;
        }
        else {
            return false;
        }
    }

    bool SteeringServo::deserialize(uint8_t *data, uint16_t size) {
        if(size >= 7u) {
            auto pulse_width_left = uint16_t(data[0] | (data[1] << 8));
            auto pulse_width_center = uint16_t(data[2] | (data[3] << 8));
            auto pulse_width_right = uint16_t(data[4] | (data[5] << 8));
            decltype(steering_servo)::Configuration configuration{
                pulse_width_left, pulse_width_center, pulse_width_right, pulse_width_center, true};
            steering_servo.setConfiguration(configuration);
            steering_servo.setPosition(current_steering_angle);  // Enforce new pulse width limits.
            input_channel = data[6];
            // Reset changed and save flags (we got this configuration from EEPROM, so it's not a
            // change).
            steering_servo.resetConfigurationChanged();
            input_channel_changed = false;
            save_required = false;
            return true;
        }
        else {
            return false;
        }
    }

}  // namespace earth_rover_vcu