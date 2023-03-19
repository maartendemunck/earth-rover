// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__HMI_STATE__
#define __EARTH_ROVER__HMI_STATE__

#include "eeprom-configuration-database.hpp"
#include "input-devices.hpp"

namespace earth_rover {

    class HmiState {

      private:
        enum class ConfigRecordType : uint8_t {
            SteeringStickConfig,
            ThrottleStickConfig,
            GearboxStickConfig,
            RadioConfig
        };
        static constexpr uint8_t config_record_type_count{4u};
        static constexpr uint16_t config_record_size{32u};
        EepromConfigDatabase<config_record_size, config_record_type_count> config_database;

        AnalogInput steering_input;
        AnalogInput throttle_input;
        DiscretizedAnalogInput gearbox_input;

      public:
        HmiState(uint16_t eeprom_block_offset, uint16_t eeprom_block_size,
                 AnalogInput steering_input, AnalogInput throttle_input,
                 DiscretizedAnalogInput gearbox_input);

        void setup();
        void spinOnce();

        void requestConfigSave();

        AnalogInput &getSteeringInput();
        AnalogInput &getThrottleInput();
        DiscretizedAnalogInput &getGearboxInput();
    };

}  // namespace earth_rover

#endif