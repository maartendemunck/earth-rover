// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__EEPROM_CONFIGURATION_DATABASE__
#define __EARTH_ROVER__EEPROM_CONFIGURATION_DATABASE__

#include "eeprom-configuration-database-object.hpp"
#include "from-to-integral.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <cstdint>

namespace earth_rover {

    template <uint16_t record_size, uint8_t record_type_count> class EepromConfigDatabase {
      private:
        static constexpr uint16_t record_data_size{record_size - 8u};

        const uint32_t eeprom_block_offset;
        const uint32_t eeprom_block_size;

        bool is_database_initialised;

        uint16_t current_record_indices[record_type_count];
        uint16_t last_record_index;
        uint32_t last_record_sequence;

        uint8_t getTypeFromRecord(uint8_t *record) {
            return record[0];
        }

        void saveTypeInRecord(uint8_t record_type, uint8_t *record) {
            record[0] = record_type;
        }

        uint32_t getSequenceFromRecord(uint8_t *record) {
            return record[2] | (record[3] << 8) | (record[4] << 16) | (record[5] << 24);
        }

        void saveSequenceInRecord(uint32_t sequence, uint8_t *record) {
            record[2] = sequence & 0x000000ff;
            record[3] = (sequence & 0x0000ff00) >> 8;
            record[4] = (sequence & 0x00ff0000) >> 16;
            record[5] = (sequence & 0xff000000) >> 24;
        }

        uint8_t *getPointerToData(uint8_t *record) {
            return &record[6];
        }

        void addChecksumToRecord(bool has_checksum, uint8_t *record) {
            if(has_checksum) {
                addFletcher16ChecksumToRecord(record);
            }
            else {
                addDummyChecksumToRecord(record);
            }
        }

        void addFletcher16ChecksumToRecord(uint8_t *record) {
            record[1] = 0xff;
            const auto checksum = calculateFletcher16Checksum(record);
            record[record_size - 2] = checksum & 0x00ff;
            record[record_size - 1] = (checksum & 0xff00) >> 8;
        }

        void addDummyChecksumToRecord(uint8_t *record) {
            record[1] = 0x00;
            record[record_size - 2] = 0xff;
            record[record_size - 1] = 0xff;
        }

        uint16_t calculateFletcher16Checksum(uint8_t *record) {
            constexpr uint32_t initial_value{0u};
            static_assert(initial_value
                                  + UINT8_MAX * ((record_size - 2) + 1) * (record_size - 2) / 2
                              <= UINT32_MAX,
                          "record too big for fletcher16 checksum algorithm");
            uint32_t sum1{initial_value}, sum2{initial_value};
            for(uint32_t index = 0; index < record_size - sizeof(uint16_t); ++index) {
                sum1 += record[index];
                sum2 += sum1;
            }
            return ((sum2 % 0xff) << 8) | (sum1 % 0xff);
        }

        bool isChecksumValid(uint8_t *record) {
            if(record[1] == 0x00) {
                return true;
            }
            else if(record[1] == 0xff) {
                const uint16_t checksum{calculateFletcher16Checksum(record)};
                if(record[record_size - 2] == (checksum & 0x00ff)
                   && record[record_size - 1] == ((checksum & 0xff00) >> 8)) {
                    return true;
                }
            }
            return false;
        }

        uint16_t findNextAvailableRecordIndex() {
            const uint16_t record_count = eeprom_block_size / record_size;
            do {
                last_record_index = (last_record_index + 1) % record_count;
            } while(!isRecordIndexAvailable(last_record_index));
            return last_record_index;
        }

        bool isRecordIndexAvailable(uint16_t record_index) {
            for(uint8_t type_index = 0; type_index < record_type_count; ++type_index) {
                if(current_record_indices[type_index] == record_index) {
                    return false;
                }
            }
            return true;
        }

      public:
        EepromConfigDatabase(uint32_t eeprom_block_offset, uint32_t eeprom_block_size)
            : eeprom_block_offset{eeprom_block_offset}, eeprom_block_size{eeprom_block_size},
              is_database_initialised{false}, last_record_index{UINT16_MAX}, last_record_sequence{
                                                                                 0u} {
            for(uint8_t record_type = 0; record_type < record_type_count; ++record_type) {
                current_record_indices[record_type] = 0xffffu;
            }
        }

        ~EepromConfigDatabase() = default;

        void initialiseDatabase() {
            const uint16_t record_count = eeprom_block_size / record_size;
            uint32_t current_record_sequences[record_type_count]{0};
            for(uint16_t record_index = 0; record_index < record_count; ++record_index) {
                uint8_t record[record_size];
                EEPROM.get(eeprom_block_offset + record_index * record_size, record);
                uint8_t record_type = getTypeFromRecord(record);
                if(record_type >= record_type_count) {
                    continue;
                }
                uint32_t record_sequence = getSequenceFromRecord(record);
                if(record_sequence != 0xffffffff && record_sequence != 0x00000000
                   && isChecksumValid(record)) {
                    if(record_sequence > last_record_sequence) {
                        last_record_index = record_index;
                        last_record_sequence = record_sequence;
                    }
                    if(record_sequence > current_record_sequences[record_type]) {
                        current_record_indices[record_type] = record_index;
                        current_record_sequences[record_type] = record_sequence;
                    }
                }
                else {
                    ;  // Skip empty and invalid records.
                }
            }
            is_database_initialised = true;
        }

        template <typename Object_t>
        bool saveObject(Object_t &object, uint8_t record_type, bool has_checksum = true) {
            using SerializationResult = EepromConfigDatabaseObject::SerializationResult;
            if(is_database_initialised == false || record_type >= record_type_count) {
                return true;
            }
            uint8_t record[record_size];
            bool record_type_exists = false;
            if(current_record_indices[record_type] != 0xffff) {
                EEPROM.get(eeprom_block_offset + current_record_indices[record_type] * record_size,
                           record);
                record_type_exists = true;
            }
            else {
                record_type_exists = false;
            }
            auto action = object.serialize(&record[6], record_data_size);
            if(!record_type_exists || action == SerializationResult::SAVE_IN_NEW_RECORD) {
                saveTypeInRecord(record_type, record);
                uint32_t sequence = ++last_record_sequence;
                saveSequenceInRecord(sequence, record);
                current_record_indices[record_type] = findNextAvailableRecordIndex();
            }
            if(action == SerializationResult::SAVE_IN_EXISTING_RECORD
               || action == SerializationResult::SAVE_IN_NEW_RECORD) {
                addChecksumToRecord(has_checksum, record);
                EEPROM.put(eeprom_block_offset + current_record_indices[record_type] * record_size,
                           record);
                object.markSaved();
                return false;
            }
            else {
                return true;
            }
        }

        template <typename Object_t>
        bool saveObjectIfRequired(Object_t &object, uint8_t record_type, bool has_checksum = true) {
            if(object.isSaveRequested()) {
                bool error = saveObject(object, record_type, has_checksum);
                if(error == false) {
                    object.markSaved();
                }
                return error;
            }
            else {
                return false;
            }
        }

        template <typename Object_t>
        bool loadObject(Object_t &object, uint8_t record_type, bool has_checksum = true) {
            if(is_database_initialised == false || record_type >= record_type_count
               || current_record_indices[record_type] == 0xffff) {
                return true;
            }
            uint8_t record[record_size];
            EEPROM.get(eeprom_block_offset + current_record_indices[record_type] * record_size,
                       record);
            if(isChecksumValid(record)) {
                bool error = object.deserialize(&record[6], record_data_size);
                if(error == false) {
                    object.markSaved();
                }
                return error;
            }
            else {
                return true;
            }
        }
    };

}  // namespace earth_rover

#endif