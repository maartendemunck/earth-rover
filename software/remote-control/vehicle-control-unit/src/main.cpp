// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include <Arduino.h>
#include <i2c_t3.h>

#include "adafruit-bno055-wrapper.hpp"
#include "lighting.hpp"
#include "neogps-wrapper.hpp"
#include "position-encoder.hpp"
#include "powertrain.hpp"
#include "radio-configuration.hpp"
#include "steering-servo.hpp"
#include "vcu-communicator.hpp"
#include "vcu.hpp"

// #define SERIAL_DEBUG

constexpr uint8_t spi_sck_pin = 14u;  // Use the default SCK pin 13 as status LED.

auto makeVcu() {
    constexpr uint16_t config_eeprom_block_offset = 0u;
    constexpr uint16_t config_eeprom_block_size = 2048u;
    constexpr uint8_t steering_servo_pin = 20u;
    constexpr uint8_t esc_pin = 21u;
    constexpr uint8_t gearbox_servo_pin = 22u;
    constexpr uint8_t head_lamp_pin = 6u;
    constexpr uint8_t tail_lamp_pin = 2u;
    constexpr uint8_t stop_lamp_pin = 3u;
    constexpr uint8_t turn_signal_left_pin = 4u;
    constexpr uint8_t turn_signal_right_pin = 5u;
    constexpr uint8_t hall_sensor_a_pin = 7u;
    constexpr uint8_t hall_sensor_b_pin = 8u;
    constexpr uint16_t pulses_per_km = 23000u;
    constexpr auto &imu_i2c = Wire;
    constexpr uint8_t imu_i2c_scl_pin = 19u;
    constexpr uint8_t imu_i2c_sda_pin = 18u;
    constexpr auto &gps_serial = Serial1;
    constexpr uint8_t gps_serial_rx_pin = 0u;
    constexpr uint8_t gps_serial_tx_pin = 1u;
    earth_rover::SteeringServo steering_servo{steering_servo_pin};
    earth_rover::Powertrain powertrain{esc_pin, gearbox_servo_pin};
    earth_rover::Lighting lighting{head_lamp_pin, tail_lamp_pin, stop_lamp_pin,
                                   turn_signal_right_pin, turn_signal_left_pin};
    earth_rover::PositionEncoder<hall_sensor_a_pin, hall_sensor_b_pin> position_encoder{
        pulses_per_km, 0u, 0u};
    earth_rover::AdafruitBNO055Wrapper<decltype(imu_i2c)> imu{imu_i2c, imu_i2c_scl_pin,
                                                              imu_i2c_sda_pin};
    earth_rover::NeoGpsWrapper<decltype(gps_serial)> gps{gps_serial, gps_serial_rx_pin,
                                                         gps_serial_tx_pin};
    earth_rover::RadioConfig radio_config;
    earth_rover::Vcu<decltype(steering_servo), decltype(powertrain), decltype(lighting),
                     decltype(position_encoder), decltype(imu), decltype(gps),
                     decltype(radio_config)>
        vcu{config_eeprom_block_offset,
            config_eeprom_block_size,
            std::move(steering_servo),
            std::move(powertrain),
            std::move(lighting),
            std::move(position_encoder),
            std::move(imu),
            std::move(gps),
            std::move(radio_config)};
    return vcu;
}

auto vcu = makeVcu();

auto makeCommunicator(decltype(vcu) &vcu) {
    constexpr uint8_t rf24_ce_pin = 10u;
    constexpr uint8_t rf24_csn_pin = 15u;
    earth_rover::VcuCommunicator<decltype(vcu)> communicator{rf24_ce_pin, rf24_csn_pin, vcu};
    return communicator;
}

auto communicator = makeCommunicator(vcu);
// auto configuration_manager = earth_rover::makeVcuConfigManager(
//     steering_servo, powertrain, position_encoder, imu, radio_config, 0u, 2048u);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
// Setup debug console (if enabled).
#ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial) {
        ;
    }
#endif
    SPI.setSCK(spi_sck_pin);
    // configuration_manager.setup();
    vcu.setup();
    communicator.setup();
    digitalWrite(LED_BUILTIN, 0);
}

void loop() {
    communicator.spinOnce();
    vcu.spinOnce();
    // configuration_manager.spinOnce();
}