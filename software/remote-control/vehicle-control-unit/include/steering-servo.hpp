// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__STEERING_SERVO__
#define __EARTH_ROVER__STEERING_SERVO__

#include "configured-servo.hpp"
#include "eeprom-configuration-database-object.hpp"
#include <cstdint>

namespace earth_rover {

    class SteeringServo : public EepromConfigDatabaseObject {

      public:
        struct Config {
            uint16_t pulse_width_left;
            uint16_t pulse_width_center;
            uint16_t pulse_width_right;
        };

      private:
        uint8_t pin_number;
        ConfiguredServo steering_servo;
        int16_t current_steering_angle;

      public:
        SteeringServo(uint8_t pin_number);
        virtual ~SteeringServo() = default;

        void setup();
        void spinOnce();

        void setNormalizedSteeringAngle(int16_t steering_angle);

        void configureSteeringServo(uint16_t pulse_width_left, uint16_t pulse_width_center,
                                    uint16_t pulse_width_right);
        Config getConfig();
        virtual SerializationResult serialize(uint8_t *data, uint16_t size) override;
        virtual bool deserialize(uint8_t *data, uint16_t size) override;
    };

}  // namespace earth_rover

#endif