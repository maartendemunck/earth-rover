// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "car-state.hpp"
#include <Arduino.h>

namespace earth_rover {

    void CarState::checkSavedConfig() {
        config_received_from_vcu = steering_servo.isConfigAvailable() && esc.isConfigAvailable()
                                   && gearbox_servo.isConfigAvailable()
                                   && radio.isConfigAvailable();
    }

    CarState::CarState(ServoConfigParams steering_servo_defaults, ServoConfigParams esc_defaults,
                       TwoSpeedGearboxServoConfigParams gearbox_servo_defaults,
                       RadioConfigParams radio_defaults)
        : steering_servo{std::move(steering_servo_defaults)}, esc{std::move(esc_defaults)},
          gearbox_servo{std::move(gearbox_servo_defaults)}, radio{std::move(radio_defaults)},
          speedometer{1000u}, orientation{1000u}, location{5000u}, altitude{5000u} {
        ;
    }

    void CarState::setup() {
        ;
    }

    void CarState::spinOnce() {
        ;
    }

    bool CarState::isConfigAvailable() {
        return config_received_from_vcu;
    }

    bool CarState::isCurrentConfigSaved() {
        return steering_servo.isCurrentConfigSavedInVcu() && esc.isCurrentConfigSavedInVcu()
               && gearbox_servo.isCurrentConfigSavedInVcu() && radio.isCurrentConfigSavedInVcu();
    }

    void CarState::requestConfigSave() {
        // Prevent overwriting the config before the VCU config is received.
        if(config_received_from_vcu) {
            config_save_requested = true;
        }
    }

    bool CarState::isConfigSaveRequested() {
        return config_save_requested;
    }

    void CarState::markConfigSaved() {
        config_save_requested = false;
    }

    void CarState::setSteeringInput(int16_t steering) {
        steering_servo.setCurrentPosition(limit_value(steering, int16_t(-1000), int16_t(1000)));
        cancelTurnSignalsIfNeeded(steering);
    }

    int16_t CarState::getCurrentSteeringPosition() {
        return steering_servo.getCurrentPosition();
    }

    void CarState::setSteerLeftPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{steering_servo.getCurrentConfig()};
        current_config.pulse_width_minimum = pulse_width;
        steering_servo.setCurrentConfig(current_config);
    }

    void CarState::setSteerCenterPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{steering_servo.getCurrentConfig()};
        current_config.pulse_width_center = pulse_width;
        steering_servo.setCurrentConfig(current_config);
    }

    void CarState::setSteerRightPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{steering_servo.getCurrentConfig()};
        current_config.pulse_width_maximum = pulse_width;
        steering_servo.setCurrentConfig(current_config);
    }

    void CarState::setSavedSteeringConfig(const ServoConfigParams &saved_config) {
        steering_servo.setSavedConfig(saved_config, true);
        checkSavedConfig();
    }

    bool CarState::isSteeringConfigAvailable() {
        return steering_servo.isConfigAvailable();
    }

    const ServoConfigParams &CarState::getSteeringConfig() {
        return steering_servo.getCurrentConfig();
    }

    bool CarState::isCurrentSteeringConfigSaved() {
        return steering_servo.isCurrentConfigSavedInVcu();
    }

    void CarState::markSteeringConfigSaved() {
        steering_servo.markCurrentConfigSaved();
    }

    void CarState::setThrottleInput(int16_t throttle) {
        esc.setCurrentPosition(limit_value(throttle, int16_t(-1000), int16_t(1000)));
    }

    int16_t CarState::getCurrentThrottlePosition() {
        return esc.getCurrentPosition();
    }

    void CarState::setFullBackwardsPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{esc.getCurrentConfig()};
        current_config.pulse_width_minimum = pulse_width;
        esc.setCurrentConfig(current_config);
    }

    void CarState::setStopPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{esc.getCurrentConfig()};
        current_config.pulse_width_center = pulse_width;
        esc.setCurrentConfig(current_config);
    }

    void CarState::setFullForwardPulseWidth(uint16_t pulse_width) {
        ServoConfigParams current_config{esc.getCurrentConfig()};
        current_config.pulse_width_maximum = pulse_width;
        esc.setCurrentConfig(current_config);
    }

    void CarState::setSavedThrottleConfig(const ServoConfigParams &saved_config) {
        esc.setSavedConfig(saved_config, true);
        checkSavedConfig();
    }

    bool CarState::isThrottleConfigAvailable() {
        return esc.isConfigAvailable();
    }

    const ServoConfigParams &CarState::getThrottleConfig() {
        return esc.getCurrentConfig();
    }

    bool CarState::isCurrentThrottleConfigSaved() {
        return esc.isCurrentConfigSavedInVcu();
    }

    void CarState::markThrottleConfigSaved() {
        esc.markCurrentConfigSaved();
    }

    void CarState::setCurrentGear(int16_t gear) {
        gearbox_servo.setCurrentGear(gear);
    }

    int8_t CarState::getCurrentGear() {
        return gearbox_servo.getCurrentGear();
    }

    void CarState::setGearPulseWidth(int8_t gear, uint16_t pulse_width) {
        TwoSpeedGearboxServoConfigParams current_config{gearbox_servo.getCurrentConfig()};
        current_config.setPulseWidthForSingleGear(gear, pulse_width);
        gearbox_servo.setCurrentConfig(current_config);
    }

    void CarState::setSavedGearboxConfig(const TwoSpeedGearboxServoConfigParams &Saved_config,
                                         bool complete) {
        gearbox_servo.setSavedConfig(Saved_config, complete);
        checkSavedConfig();
    }

    bool CarState::isGearboxConfigAvailable() {
        return gearbox_servo.isConfigAvailable();
    }

    const TwoSpeedGearboxServoConfigParams &CarState::getGearboxConfig() {
        return gearbox_servo.getCurrentConfig();
    }

    bool CarState::isCurrentGearboxConfigSaved() {
        return gearbox_servo.isCurrentConfigSavedInVcu();
    }

    void CarState::markGearboxConfigSaved() {
        gearbox_servo.markCurrentConfigSaved();
    }

    void CarState::setHmiRadioPower(uint8_t power_level) {
        RadioConfigParams current_config{radio.getCurrentConfig()};
        current_config.tx_power = power_level;
        radio.setCurrentConfig(current_config);
    }

    void CarState::setVcuRadioPower(uint8_t power_level) {
        RadioConfigParams current_config{radio.getCurrentConfig()};
        current_config.rx_power = power_level;
        radio.setCurrentConfig(current_config);
    }

    void CarState::setSavedRadioConfig(const RadioConfigParams &Saved_config) {
        radio.setSavedConfig(Saved_config);
        checkSavedConfig();
    }

    bool CarState::isRadioConfigAvailable() {
        return radio.isConfigAvailable();
    }

    const RadioConfigParams &CarState::getRadioConfig() {
        return radio.getCurrentConfig();
    }

    bool CarState::isCurrentRadioConfigSaved() {
        return radio.isCurrentConfigSavedInVcu();
    }

    void CarState::markRadioConfigSaved() {
        radio.markCurrentConfigSaved();
    }

    void CarState::setTurnSignalRight(bool state) {
        lighting.turn_signal_right = state;
    }

    void CarState::setTurnSignalLeft(bool state) {
        lighting.turn_signal_left = state;
    }

    void CarState::cancelTurnSignalsIfNeeded(int16_t steering) {
        if(lighting.turn_signal_right && !lighting.turn_signal_left) {
            if(steering > 500) {
                turn_signal_free = true;
            }
            else if(turn_signal_free && steering < 100) {
                setTurnSignalRight(false);
                turn_signal_free = false;
                turn_signal_right_cancelled = true;
            }
        }
        else if(lighting.turn_signal_left && !lighting.turn_signal_right) {
            if(steering < -500) {
                turn_signal_free = true;
            }
            else if(turn_signal_free && steering > -100) {
                setTurnSignalLeft(false);
                turn_signal_free = false;
                turn_signal_left_cancelled = true;
            }
        }
        else if(!lighting.turn_signal_right && !lighting.turn_signal_left) {
            turn_signal_free = false;
        }
    }

    bool CarState::getTurnSignalRightCancelled(bool reset) {
        auto return_value = turn_signal_right_cancelled;
        if(reset) {
            turn_signal_right_cancelled = false;
        }
        return return_value;
    }

    bool CarState::getTurnSignalLeftCancelled(bool reset) {
        auto return_value = turn_signal_left_cancelled;
        if(reset) {
            turn_signal_left_cancelled = false;
        }
        return return_value;
    }

    void CarState::setDippedBeam(bool state) {
        lighting.dipped_beam = state;
    }

    void CarState::setHighBeam(bool state) {
        lighting.high_beam = state;
    }

    void CarState::setHazardFlashers(bool state) {
        lighting.hazard_flashers = state;
    }

    const CarState::Lighting &CarState::getRequestedLightingState() {
        return lighting;
    }

    void CarState::setSpeedometer(const Speedometer &value, bool valid) {
        speedometer.set(value, valid);
    }

    bool CarState::isSpeedometerUpdated() {
        return speedometer.isUpdated();
    }

    SensorState<CarState::Speedometer>::Measurement CarState::getSpeedometer(bool reset_updated) {
        return speedometer.get(reset_updated);
    }

    void CarState::setOrientation(const Orientation &value, bool valid) {
        orientation.set(value, valid);
    }

    bool CarState::isOrientationUpdated() {
        return orientation.isUpdated();
    }

    SensorState<CarState::Orientation>::Measurement CarState::getOrientation(bool reset_updated) {
        return orientation.get(reset_updated);
    }

    void CarState::setLocation(const Location &value, bool valid) {
        location.set(value, valid);
    }

    bool CarState::isLocationUpdated() {
        return location.isUpdated();
    }

    SensorState<CarState::Location>::Measurement CarState::getLocation(bool reset_updated) {
        return location.get(reset_updated);
    }

    void CarState::setAltitude(int32_t value, bool valid) {
        altitude.set(value, true);
    }

    bool CarState::isAltitudeUpdated() {
        return altitude.isUpdated();
    }

    SensorState<int32_t>::Measurement CarState::getAltitude(bool reset_updated) {
        return altitude.get(reset_updated);
    }

}  // namespace earth_rover