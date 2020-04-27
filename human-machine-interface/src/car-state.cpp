//! Car state (digital twin) for the Earth Rover (implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#include "car-state.hpp"
#include <Arduino.h>

namespace earth_rover_hmi {

    CarState::CarState(ServoConfigParams steering_servo_defaults, ServoConfigParams esc_defaults,
                       GearboxServoConfigParams<0, 1, 2> gearbox_servo_defaults,
                       RadioConfigParams radio_defaults)
        : configuration_available{false}, configuration_save_request{false},
          steering_servo{std::move(steering_servo_defaults)}, esc{std::move(esc_defaults)},
          gearbox_servo{std::move(gearbox_servo_defaults)}, radio{std::move(radio_defaults)},
          speedometer{1000u}, orientation{1000u}, location{5000u}, altitude{5000u} {
        ;
    }

    void CarState::setSteeringInput(int16_t steering) {
        steering_servo.setCurrentPosition(limit_value(steering, int16_t(-1000), int16_t(1000)));
        if(lighting.turn_signal_right && !lighting.turn_signal_left) {
            // If the steering input is moved more than 50% right, unlock the right turn signal.
            if(steering > 500 && turn_signal_free != 1) {
                turn_signal_free = 1;
            }
            // If the steering input is moved back to the center (less than 10% right), shut off the
            // right turn signal.
            else if(turn_signal_free == 1 && steering < 100) {
                setTurnSignalRight(false);
                turn_signal_free = 0;
                turn_signal_right_cancelled = true;
            }
        }
        else if(lighting.turn_signal_left && !lighting.turn_signal_right) {
            // If the steering input is moved more than 50% left, unlock the left turn signal.
            if(steering < -500 && turn_signal_free != -1) {
                turn_signal_free = -1;
            }
            // If the steering input is moved back to the center (less than 10% left), shut off the
            // left turn signal.
            else if(turn_signal_free == -1 && steering > -100) {
                setTurnSignalLeft(false);
                turn_signal_free = 0;
                turn_signal_left_cancelled = true;
            }
        }
        else if(!lighting.turn_signal_right && !lighting.turn_signal_left) {
            // If no turn signal is active, deactivate the auto shut off function.
            if(turn_signal_free != 0) {
                turn_signal_free = 0;
            }
        }
    }

    void CarState::setSteeringInputChannel(uint8_t input_channel) {
        ServoConfigParams current_configuration{steering_servo.getCurrentConfiguration()};
        current_configuration.input_channel = input_channel;
        steering_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setSteerLeftPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{steering_servo.getCurrentConfiguration()};
        current_configuration.pulse_width_minimum = pulse_width;
        steering_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setSteerCenterPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{steering_servo.getCurrentConfiguration()};
        current_configuration.pulse_width_center = pulse_width;
        steering_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setSteerRightPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{steering_servo.getCurrentConfiguration()};
        current_configuration.pulse_width_maximum = pulse_width;
        steering_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setThrottleInput(int16_t throttle) {
        esc.setCurrentPosition(limit_value(throttle, int16_t(-1000), int16_t(1000)));
    }

    void CarState::setThrottleInputChannel(uint8_t input_channel) {
        ServoConfigParams current_configuration{esc.getCurrentConfiguration()};
        current_configuration.input_channel = input_channel;
        esc.setCurrentConfiguration(current_configuration);
    }

    void CarState::setFullBackwardsPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{esc.getCurrentConfiguration()};
        current_configuration.pulse_width_minimum = pulse_width;
        esc.setCurrentConfiguration(current_configuration);
    }

    void CarState::setStopPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{esc.getCurrentConfiguration()};
        current_configuration.pulse_width_center = pulse_width;
        esc.setCurrentConfiguration(current_configuration);
    }

    void CarState::setFullForwardPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_configuration{esc.getCurrentConfiguration()};
        current_configuration.pulse_width_maximum = pulse_width;
        esc.setCurrentConfiguration(current_configuration);
    }

    void CarState::setGearboxInput(int16_t gearbox) {
        // If the input stick is moved back to the center zone, unlock the gearbox.
        if(gearbox_locked && gearbox_locked * gearbox < 250) {
            gearbox_locked = 0;
        }
        // If the gearbox is unlocked and the input stick is moved to the end of the range, shift.
        if(gearbox < -750 && !gearbox_locked) {
            gearbox_servo
                .shiftDown();  // The shiftDown function will take care of the available gear range.
            gearbox_locked = -1;
        }
        else if(gearbox > 750 && !gearbox_locked) {
            gearbox_servo
                .shiftUp();  // The shiftUp function will take care of the available gear range.
            gearbox_locked = 1;
        }
    }

    void CarState::setGearboxInputChannel(uint8_t input_channel) {
        GearboxServoConfigParams<0, 1, 2> current_configuration{
            gearbox_servo.getCurrentConfiguration()};
        current_configuration.input_channel = input_channel;
        gearbox_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setGearPulseWidth(int8_t gear, uint16_t pulse_width) {
        GearboxServoConfigParams<0, 1, 2> current_configuration{
            gearbox_servo.getCurrentConfiguration()};
        current_configuration.setPulseWidth(gear, pulse_width);
        gearbox_servo.setCurrentConfiguration(current_configuration);
    }

    void CarState::setHmiRadioPower(uint8_t power_level) {
        RadioConfigParams current_configuration{radio.getCurrentConfiguration()};
        current_configuration.tx_power = power_level;
        radio.setCurrentConfiguration(current_configuration);
    }

    void CarState::setVcuRadioPower(uint8_t power_level) {
        RadioConfigParams current_configuration{radio.getCurrentConfiguration()};
        current_configuration.rx_power = power_level;
        radio.setCurrentConfiguration(current_configuration);
    }

    void CarState::setTurnSignalRight(bool state) { lighting.turn_signal_right = state; }

    bool CarState::getTurnSignalRightCancelled(bool reset) {
        auto return_value = turn_signal_right_cancelled;
        if(reset) {
            turn_signal_right_cancelled = false;
        }
        return return_value;
    }

    void CarState::setTurnSignalLeft(bool state) { lighting.turn_signal_left = state; }

    bool CarState::getTurnSignalLeftCancelled(bool reset) {
        auto return_value = turn_signal_left_cancelled;
        if(reset) {
            turn_signal_left_cancelled = false;
        }
        return return_value;
    }

    void CarState::setDippedBeam(bool state) { lighting.dipped_beam = state; }

    void CarState::setHighBeam(bool state) { lighting.high_beam = state; }

    void CarState::setHazardFlashers(bool state) { lighting.hazard_flashers = state; }

}  // namespace earth_rover_hmi