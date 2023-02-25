// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__INPUT_DEVICES__
#define __EARTH_ROVER__INPUT_DEVICES__

#include "eeprom-configuration-database-object.hpp"
#include <cstdint>

namespace earth_rover {

    class AnalogInput : public EepromConfigDatabaseObject {
        using SerializationResult = EepromConfigDatabaseObject::SerializationResult;

      private:
        uint8_t pin;
        bool calibrate_center;
        uint16_t analog_minimum = 0;
        uint16_t analog_center = 511;
        uint16_t analog_maximum = 1023;

      public:
        enum class CalibrationPosition : uint8_t {
            INVALID = 0x00,
            MINIMUM = 0x01,
            CENTER = 0x02,
            MAXIMUM = 0x03
        };

        AnalogInput(uint8_t pin, bool calibrate_center);
        virtual ~AnalogInput() = default;

        virtual int16_t readPosition() const;
        bool calibratePosition(CalibrationPosition position);

        virtual SerializationResult serialize(uint8_t *data, uint16_t size) override;
        virtual bool deserialize(uint8_t *data, uint16_t size) override;
    };

    class DiscretizedAnalogInput : public AnalogInput {

      private:
        uint8_t number_of_discrete_positions;

      public:
        DiscretizedAnalogInput(uint8_t pin, uint8_t number_of_discrete_positions);
        virtual ~DiscretizedAnalogInput() = default;

        virtual int16_t readPosition() const override;
    };

}  // namespace earth_rover

#endif