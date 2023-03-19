// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__POWERTRAIN__
#define __EARTH_ROVER__POWERTRAIN__

#include "configured-servo.hpp"
#include "eeprom-configuration-database-object.hpp"
#include <Arduino.h>
#include <cstdint>

namespace earth_rover {

    class Powertrain : public EepromConfigDatabaseObject {

      public:
        struct Config {
            struct {
                uint16_t pulse_width_reverse;
                uint16_t pulse_width_stop;
                uint16_t pulse_width_forward;
            } esc;
            struct {
                uint16_t pulse_width_low;
                uint16_t pulse_width_high;
            } gearbox;
        };

      private:
        uint8_t esc_pin_number;
        ConfiguredServo esc;
        uint8_t gearbox_servo_pin_number;
        ConfiguredServo gearbox_servo;
        uint16_t gearbox_pulse_widths[3];
        int16_t current_throttle_setting;
        int8_t current_gear;
        int8_t requested_gear;
        bool is_driving;
        elapsedMillis time_since_driving;

      public:
        Powertrain(uint8_t esc_pin_number, uint8_t gearbox_servo_pin_number);
        virtual ~Powertrain() = default;

        void setup();
        void spinOnce();

        void setNormalizedThrottleSetting(int16_t throttle_setting);
        void setGear(int8_t gear);

        void configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop,
                          uint16_t pulse_width_forward);
        void configureGearboxServo(uint16_t pulse_width_low, uint16_t pulse_width_high);
        void configureGearboxServo(int8_t gear, uint16_t pulse_width);
        Config getConfig();

        virtual SerializationResult serialize(uint8_t *data, uint16_t size) override;
        virtual bool deserialize(uint8_t *data, uint16_t size) override;
    };

}  // namespace earth_rover

#endif
