// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__EEPROM_CONFIGURATION_DATABASE_OBJECT__
#define __EARTH_ROVER__EEPROM_CONFIGURATION_DATABASE_OBJECT__

#include <cstdint>

namespace earth_rover {

    class EepromConfigDatabaseObject {
      private:
        bool is_changed{false};
        bool is_save_required{false};

      public:
        enum class SerializationResult : uint8_t {
            SAVE_IN_NEW_RECORD = 0,
            SAVE_IN_EXISTING_RECORD = 1,
            ERROR = 2
        };

        EepromConfigDatabaseObject() {
            ;
        }

        virtual ~EepromConfigDatabaseObject() = default;

        void markChanged() {
            is_changed = true;
        }
        bool isChanged() {
            return is_changed;
        }
        void requestSaveIfChanged() {
            if(is_changed) {
                is_save_required = true;
            }
        }
        virtual bool isSaveRequested() {
            return is_save_required;
        }
        void markSaved() {
            is_changed = false;
            is_save_required = false;
        }

        virtual SerializationResult serialize(uint8_t *data, uint16_t size) = 0;
        virtual bool deserialize(uint8_t *data, uint16_t size) = 0;
    };

}  // namespace earth_rover

#endif