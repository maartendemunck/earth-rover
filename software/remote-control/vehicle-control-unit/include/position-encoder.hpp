// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__POSITION_ENCODER__
#define __EARTH_ROVER__POSITION_ENCODER__

#include "eeprom-configuration-database-object.hpp"
#include "isr-wrapper.hpp"
#include <Arduino.h>
#include <cstdint>

namespace earth_rover {

    template <uint8_t hall_sensor_a_pin, uint8_t hall_sensor_b_pin>
    class PositionEncoder : public EepromConfigDatabaseObject {

      private:
        ISRWrapper<digitalPinToInterrupt(hall_sensor_a_pin)> isr_wrapper;
        const uint16_t pulses_per_km;
        volatile uint32_t odometer;
        uint32_t odometer_eeprom;
        volatile uint32_t tripmeter;
        elapsedMicros since_last_pulse;
        static constexpr uint8_t available_intervals = 8;
        volatile uint32_t between_pulses[available_intervals];
        static constexpr uint16_t serialized_resolution{16};

      public:
        PositionEncoder(uint16_t pulses_per_km, uint32_t odometer = 0u, uint32_t tripmeter = 0u)
            : pulses_per_km{pulses_per_km}, odometer{odometer},
              odometer_eeprom{odometer}, tripmeter{tripmeter} {
            ;
        }

        void setup() {
            pinMode(hall_sensor_a_pin, INPUT);
            pinMode(hall_sensor_b_pin, INPUT);
            if(!isr_wrapper.attachISR(
                   std::bind(
                       &PositionEncoder<hall_sensor_a_pin, hall_sensor_b_pin>::interruptHandler,
                       this),
                   FALLING)) {
                ;  // Unable to attach ISR handler.
            }
        }

        void spinOnce() {
            ;
        }

        uint16_t getSpeedInMeterPerHour() const {
            uint32_t numerator
                = (3600u * 1000000u / pulses_per_km) * 1000u;  // convert elapsedMicros to m/h.
            constexpr uint32_t minimum_interval = 200000;
            uint32_t since_last_pulse_const = since_last_pulse;
            if(since_last_pulse_const > between_pulses[odometer % available_intervals] * 2) {
                return int16_t(numerator / since_last_pulse_const);
            }
            else {
                uint32_t measured_interval = 0;
                int8_t measured_pulses = 0;
                while(measured_pulses < available_intervals
                      && measured_interval < minimum_interval) {
                    measured_interval
                        += between_pulses[(odometer - measured_pulses) % available_intervals];
                    ++measured_pulses;
                }
                return int16_t(numerator / (measured_interval / measured_pulses));
            }
        }

        uint32_t getOdometerInMeter() const {
            return odometer * 1000 / pulses_per_km;
        }

        uint32_t getTripmeterInMeter() const {
            return tripmeter * 1000 / pulses_per_km;
        }

        void resetTripmeter() {
            tripmeter = 0;
        }

        virtual bool isSaveRequested() override {
            return (EepromConfigDatabaseObject::isSaveRequested()
                    || (odometer - odometer_eeprom) > 16u);
        }

        // The odometer is serialized using the technique described in
        // https://arduino.stackexchange.com/questions/16301/ using (size - 2) counters and
        // adds a 16 bit (uint16) offset.

        SerializationResult serialize(uint8_t *buffer, uint16_t size) {
            // Check size of the serialized record to prevent buffer overflows.
            if(size < 3) {
                return SerializationResult::ERROR;
            }
            uint16_t counter_range = ((size - 2u) * UINT8_MAX + 1);
            // Calculate values.
            uint32_t value = odometer / serialized_resolution;
            uint16_t offset = value / counter_range;
            uint16_t counter = value % counter_range;
            uint8_t base = counter / (size - 2u);
            uint8_t augmented = counter % (size - 2u);
            // Check whether we'd prefer to update the existing record or to write a new one.
            bool write_new_record = false;
            if(offset != (buffer[0] | (buffer[1] << 8))) {
                write_new_record = true;
                buffer[0] = offset & 0x00ff;
                buffer[1] = (offset & 0xff00) >> 8;
            }
            // Compose record.
            for(uint8_t index = 0; index < size - 2u; ++index) {
                buffer[2 + index] = index < augmented ? base + 1 : base;
            }
            odometer_eeprom = value * serialized_resolution;
            return write_new_record ? SerializationResult::SAVE_IN_NEW_RECORD
                                    : SerializationResult::SAVE_IN_EXISTING_RECORD;
        }

        bool deserialize(uint8_t *buffer, uint16_t size) {
            // Check size of the serialized record to prevent buffer overflows.
            if(size < 3) {
                return true;
            }
            // Restore odometer value and set odometer.
            uint16_t offset = buffer[0] | (buffer[1] << 8);
            uint8_t base = buffer[size - 1];
            uint8_t augmented = 0;
            while(buffer[2 + augmented] > base) {
                ++augmented;
            }
            uint32_t value
                = offset * ((size - 2u) * UINT8_MAX + 1) + (base * (size - 2u)) + augmented;
            odometer_eeprom = value * serialized_resolution;
            odometer = odometer_eeprom;
            // Odometer successfully restored.
            return false;
        }

      private:
        // The interrupt handler is called on the falling edge of the first Hall effect sensor.
        void interruptHandler() {
            ++odometer;
            ++tripmeter;
            between_pulses[odometer % available_intervals] = since_last_pulse;
            since_last_pulse -= between_pulses[odometer % available_intervals];
            if(digitalRead(hall_sensor_b_pin) == LOW) {
                ;  // Driving forward.
            }
            else {
                ;  // Driving backward.
            }
        }
    };

}  // namespace earth_rover

#endif