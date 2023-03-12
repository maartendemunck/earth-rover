// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "radio-configuration.hpp"

namespace earth_rover {

    bool RadioConfig::setHmiRadioPowerLevel(uint8_t power_level) {
        if(power_level >= 0 && power_level <= 3) {
            if(power_level != hmi_radio_power_level) {
                hmi_radio_power_level = power_level;
                markChanged();
            }
            return true;
        }
        else {
            return false;
        }
    }

    uint8_t RadioConfig::getHmiRadioPowerLevel() {
        return hmi_radio_power_level;
    }

    bool RadioConfig::setVcuRadioPowerLevel(uint8_t power_level) {
        if(power_level >= 0 && power_level <= 3) {
            if(power_level != vcu_radio_power_level) {
                vcu_radio_power_level = power_level;
                markChanged();
            }
            return false;
        }
        else {
            return true;
        }
    }

    uint8_t RadioConfig::getVcuRadioPowerLevel() {
        return vcu_radio_power_level;
    }

    RadioConfig::SerializationResult RadioConfig::serialize(uint8_t *data, uint16_t size) {
        if(size >= 2u) {
            data[0] = hmi_radio_power_level;
            data[1] = vcu_radio_power_level;
            for(unsigned index = 2; index < size; ++index) {
                data[index] = 0xff;
            }
            return SerializationResult::SAVE_IN_NEW_RECORD;
        }
        else {
            return SerializationResult::ERROR;
        }
    }

    bool RadioConfig::deserialize(uint8_t *data, uint16_t size) {
        if(size >= 2u) {
            hmi_radio_power_level = data[0];
            vcu_radio_power_level = data[1];
            return false;
        }
        else {
            return true;
        }
    }

}  // namespace earth_rover