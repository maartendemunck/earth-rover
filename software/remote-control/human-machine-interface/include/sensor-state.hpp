// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__SENSOR_STATE__
#define __EARTH_ROVER__SENSOR_STATE__

#include <Arduino.h>

namespace earth_rover {

    template <typename SensorMeasurement_t> class SensorState {

      public:
        struct Measurement {
            SensorMeasurement_t data;
            bool valid;
        };

      private:
        const unsigned long maximum_age_ms;
        SensorMeasurement_t measurement;
        bool valid;
        bool updated;
        elapsedMillis since_update;

      public:
        SensorState(unsigned long maximum_age_ms) : maximum_age_ms{maximum_age_ms}, valid{false} {
            ;
        }

        ~SensorState() = default;

        void set(SensorMeasurement_t new_measurement, bool new_valid = true) {
            if(new_valid == true) {
                since_update = 0;
            }
            if(new_measurement != measurement || new_valid != valid) {
                measurement = new_measurement;
                valid = new_valid;
                updated = true;
            }
        }

        Measurement get(bool reset_updated = true) {
            checkMaximumAge();
            Measurement result;
            result.data = measurement;
            result.valid = valid;
            if(reset_updated) {
                updated = false;
            }
            return result;
        }

        bool isUpdated() {
            checkMaximumAge();
            return updated;
        }

      private:
        void checkMaximumAge() {
            if(valid == true && maximum_age_ms != 0 && since_update > maximum_age_ms) {
                valid = false;
                updated = true;
            }
        }
    };

}  // namespace earth_rover

#endif