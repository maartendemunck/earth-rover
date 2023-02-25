// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "hmi-state.hpp"

namespace earth_rover {

    HmiState::HmiState(uint16_t eeprom_block_offset, uint16_t eeprom_block_size,
                       AnalogInput steering_input, AnalogInput throttle_input,
                       DiscretizedAnalogInput gearbox_input)
        : config_database{eeprom_block_offset, eeprom_block_size}, steering_input{std::move(
                                                                       steering_input)},
          throttle_input{std::move(throttle_input)}, gearbox_input{std::move(gearbox_input)} {
        ;
    }

    void HmiState::setup() {
        config_database.initialiseDatabase();
        config_database.loadObject(getSteeringInput(),
                                   to_integral(ConfigRecordType::SteeringStickConfig), true);
        config_database.loadObject(getThrottleInput(),
                                   to_integral(ConfigRecordType::ThrottleStickConfig), true);
        config_database.loadObject(getGearboxInput(),
                                   to_integral(ConfigRecordType::GearboxStickConfig), true);
    }

    void HmiState::spinOnce() {
        config_database.saveObjectIfRequired(
            getSteeringInput(), to_integral(ConfigRecordType::SteeringStickConfig), true);
        config_database.saveObjectIfRequired(
            getThrottleInput(), to_integral(ConfigRecordType::ThrottleStickConfig), true);
        config_database.saveObjectIfRequired(
            getGearboxInput(), to_integral(ConfigRecordType::GearboxStickConfig), true);
    }

    void HmiState::requestConfigSave() {
        getSteeringInput().requestSaveIfChanged();
        getThrottleInput().requestSaveIfChanged();
        getGearboxInput().requestSaveIfChanged();
    }

    AnalogInput &HmiState::getSteeringInput() {
        return steering_input;
    }

    AnalogInput &HmiState::getThrottleInput() {
        return throttle_input;
    }

    DiscretizedAnalogInput &HmiState::getGearboxInput() {
        return gearbox_input;
    }

}  // namespace earth_rover
