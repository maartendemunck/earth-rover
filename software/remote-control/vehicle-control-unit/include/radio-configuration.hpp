// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__RADIO_CONFIGURATION
#define __EARTH_ROVER__RADIO_CONFIGURATION

#include "eeprom-configuration-database-object.hpp"
#include <cstdint>

namespace earth_rover {

    class RadioConfig : public EepromConfigDatabaseObject {

      private:
        uint8_t hmi_radio_power_level{1u};
        uint8_t vcu_radio_power_level{1u};

      public:
        bool setHmiRadioPowerLevel(uint8_t power_level);
        uint8_t getHmiRadioPowerLevel();
        bool setVcuRadioPowerLevel(uint8_t power_level);
        uint8_t getVcuRadioPowerLevel();

        virtual SerializationResult serialize(uint8_t *data, uint16_t size) override;
        virtual bool deserialize(uint8_t *data, uint16_t size) override;
    };

}  // namespace earth_rover

#endif