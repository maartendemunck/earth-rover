//! Sensor state (interface and template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#include <Arduino.h>

namespace earth_rover_hmi {

    //! Sensor state, adding a timestamp and a valid flag to a sensor measurement.
    /*!
     *  \tparam SensorMeasurement_t Measurement data type.
     *  \ingroup HMI
     */
    template <typename SensorMeasurement_t> class SensorState {
      public:
        //! A measurement with a flag indicating whether the measurement is (still) valid.
        /*!
         *  \ingroup HMI
         */
        struct Measurement {
            SensorMeasurement_t data;  //!< Measured value(s).
            bool valid;                //!< Valid flag.
        };

      private:
        const unsigned long maximum_age_ms;  //!< Maximum age of the measurement before it is
                                             //!< considered invalid anyway.
        SensorMeasurement_t measurement;     //!< Measured value(s).
        bool valid;                  //!< Flag indicating whether the measurement is valid or not.
        bool updated;                //!< Flag indicating whether the measurement is updated.
        elapsedMillis since_update;  //!< Time since the last update.

      public:
        //! Constructor.
        /*!
         *  \param maximum_age_ms Maximum age of the measurement (in ms) before it is considered
         * invalid.
         */
        SensorState(unsigned long maximum_age_ms) : maximum_age_ms{maximum_age_ms}, valid{false} {
            ;
        }

        //! Default constructor.
        ~SensorState() = default;

        //! Update the sensor state.
        /*!
         *  \param new_measurement New measurement.
         *  \param new_valid A flag indicating whether the new measurement is valid or not.
         */
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

        //! Get the sensor state.
        /*!
         *  \param reset_updated True to reset the updated flag, false to leave it at its current
         * value. \return A struct containing the measurement and a flag to indicate whether it's
         * valid or not.
         */
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

        //! Check whether the measurement is updated.
        /*!
         *  \return True if the measurement is updated since the last get() call with reset_updated
         * = true, false if not.
         */
        bool isUpdated() {
            checkMaximumAge();
            return updated;
        }

      private:
        //! If the measurement is too old, invalidate it (and set updated to true).
        void checkMaximumAge() {
            if(valid == true && maximum_age_ms != 0 && since_update > maximum_age_ms) {
                valid = false;
                updated = true;
            }
        }
    };

}  // namespace earth_rover_hmi