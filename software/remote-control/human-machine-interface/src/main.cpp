// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

// #define SERIAL_DEBUG

#include "car-state.hpp"
#include "hmi-communicator.hpp"
#include "hmi-state.hpp"
#include "limit-value.hpp"
#include "nextion-hmi-display.hpp"
#include "servo-state.hpp"
#include <Arduino.h>

constexpr uint8_t spi_sck_pin = 14;  // The default SCK pin 13 is used as status LED.

auto makeHmiState() {
    constexpr uint16_t config_eeprom_block_offset = 0u;
    constexpr uint16_t config_eeprom_block_size = 2048u;
    constexpr uint8_t steering_input_pin = 2;
    constexpr uint8_t throttle_input_pin = 5;
    constexpr uint8_t gearbox_input_pin = 4;
    earth_rover::AnalogInput steering_input(steering_input_pin, true);
    earth_rover::AnalogInput throttle_input(throttle_input_pin, true);
    earth_rover::DiscretizedAnalogInput gearbox_input(gearbox_input_pin, 2u);
    earth_rover::HmiState hmi_state{config_eeprom_block_offset, config_eeprom_block_size,
                                    std::move(steering_input), std::move(throttle_input),
                                    std::move(gearbox_input)};
    // TODO: Move radio settings to HmiState.
    return hmi_state;
}

auto makeCarState() {
    earth_rover::ServoConfigParams steering_servo_defaults{0u, 0u, 0u, 0u, true};
    earth_rover::ServoConfigParams throttle_defaults{3u, 0u, 0u, 0u, true};
    uint16_t gearbox_servo_pulse_widths[2]{0u, 0u};
    earth_rover::TwoSpeedGearboxServoConfigParams gearbox_servo_defaults{
        gearbox_servo_pulse_widths};
    earth_rover::RadioConfigParams radio_defaults{1u, 1u};
    earth_rover::CarState car_state{std::move(steering_servo_defaults),
                                    std::move(throttle_defaults), std::move(gearbox_servo_defaults),
                                    std::move(radio_defaults)};
    return car_state;
}

auto makeHmiCommunicator(earth_rover::CarState &car_state) {
    constexpr uint8_t rf24_ce_pin = 6;
    constexpr uint8_t rf24_csn_pin = 15;
    earth_rover::HmiCommunicator communicator{rf24_ce_pin, rf24_csn_pin, car_state};
    return communicator;
}

auto makeNextionHmiDisplay(earth_rover::CarState &car_state, earth_rover::HmiState &hmi_state) {
    earth_rover::NextionHmiDisplay<decltype(Serial2)> display{Serial2, car_state, hmi_state};
    return display;
}

auto hmi_state{makeHmiState()};
auto car_state{makeCarState()};
auto communicator{makeHmiCommunicator(car_state)};
auto display{makeNextionHmiDisplay(car_state, hmi_state)};

void setup() {
    // Initialisation started, switch on built in LED.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
#ifdef SERIAL_DEBUG
    // Setup debug console.
    Serial.begin(9600);
    while(!Serial) {
        ;
    }
#endif
    hmi_state.setup();
    car_state.setup();
    SPI.setSCK(spi_sck_pin);
    communicator.setup();
    display.setup();
    // Initialisation finished, switch off built in LED.
    digitalWrite(LED_BUILTIN, 0);
}

void loop() {
    auto steering_position = hmi_state.getSteeringInput().readPosition();
    car_state.setSteeringInput(steering_position);
    auto throttle_position = hmi_state.getThrottleInput().readPosition();
    car_state.setThrottleInput(throttle_position);
    auto current_gear = hmi_state.getGearboxInput().readPosition() + 1u;
    car_state.setCurrentGear(current_gear);
    hmi_state.spinOnce();
    car_state.spinOnce();
    communicator.spinOnce();
    display.spinOnce();
}

void serialEvent2() {
    display.receiveData();
}