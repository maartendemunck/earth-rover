// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "input-devices.hpp"
#include "limit-value.hpp"
#include <Arduino.h>

namespace earth_rover {

    AnalogInput::AnalogInput(uint8_t pin, bool calibrate_center)
        : pin{pin}, calibrate_center{calibrate_center} {
        ;
    }

    int16_t AnalogInput::readPosition() const {
        auto raw_input = analogRead(pin);
        int16_t position;
        if(calibrate_center) {
            if(raw_input < analog_center) {
                position
                    = (raw_input - analog_center) * (10000 / (analog_center - analog_minimum)) / 10;
            }
            else {
                position
                    = (raw_input - analog_center) * (10000 / (analog_maximum - analog_center)) / 10;
            }
        }
        else {
            position = (raw_input - ((analog_maximum + analog_minimum) / 2))
                       * (20000 / (analog_maximum - analog_minimum)) / 10;
        }
        return limit_value<int16_t>(position, -1000, 1000);
    }

    bool AnalogInput::calibratePosition(CalibrationPosition position) {
        if(position == CalibrationPosition::MINIMUM) {
            analog_minimum = analogRead(pin);
            markChanged();
            return false;
        }
        else if(position == CalibrationPosition::MAXIMUM) {
            analog_maximum = analogRead(pin);
            markChanged();
            return false;
        }
        else if(position == CalibrationPosition::CENTER && calibrate_center) {
            analog_center = analogRead(pin);
            markChanged();
            return false;
        }
        else {
            return true;
        }
    }

    AnalogInput::SerializationResult AnalogInput::serialize(uint8_t *data, uint16_t size) {
        if(size >= 6u) {
            data[0] = analog_minimum & 0x00ff;
            data[1] = (analog_minimum & 0xff00) >> 8;
            data[2] = analog_center & 0x00ff;
            data[3] = (analog_center & 0xff00) >> 8;
            data[4] = analog_maximum & 0x00ff;
            data[5] = (analog_maximum & 0xff00) >> 8;
            for(uint16_t index = 6; index < size; ++index) {
                data[index] = 0xff;
            }
            return SerializationResult::SAVE_IN_NEW_RECORD;
        }
        else {
            return SerializationResult::ERROR;
        }
    }

    bool AnalogInput::deserialize(uint8_t *data, uint16_t size) {
        if(size >= 6u) {
            analog_minimum = uint16_t(data[0] | (data[1] << 8));
            analog_center = uint16_t(data[2] | (data[3] << 8));
            analog_maximum = uint16_t(data[4] | (data[5] << 8));
            return false;
        }
        else {
            return true;
        }
    }

    DiscretizedAnalogInput::DiscretizedAnalogInput(uint8_t pin,
                                                   uint8_t number_of_discrete_positions)
        : AnalogInput(pin, false), number_of_discrete_positions(number_of_discrete_positions) {
        ;
    }

    int16_t DiscretizedAnalogInput::readPosition() const {
        if(number_of_discrete_positions > 1) {
            auto analog_position = AnalogInput::readPosition();
            int16_t step_size = 2000 / (number_of_discrete_positions - 1);
            return ((analog_position + 1000 + step_size / 2) / step_size);
        }
        else {
            return 0;
        }
    }

}  // namespace earth_rover