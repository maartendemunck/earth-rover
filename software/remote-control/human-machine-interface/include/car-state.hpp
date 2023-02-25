// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__CAR_STATE__
#define __EARTH_ROVER__CAR_STATE__

#include "gearbox-servo-state.hpp"
#include "limit-value.hpp"
#include "radio-state.hpp"
#include "sensor-state.hpp"
#include "servo-state.hpp"
#include <DMS.h>
#include <cstdint>

namespace {
    bool operator!=(const DMS_t &lhs, const DMS_t &rhs) {
        return (lhs.degrees != rhs.degrees || lhs.minutes != rhs.minutes
                || lhs.hemisphere != rhs.hemisphere || lhs.seconds_whole != rhs.seconds_whole
                || lhs.seconds_frac != rhs.seconds_frac);
    }
}  // namespace

namespace earth_rover {

    class CarState {

      public:
        struct Lighting {
            bool turn_signal_right;
            bool turn_signal_left;
            bool dipped_beam;
            bool high_beam;
            bool hazard_flashers;
        };

        struct Speedometer {
            float speed_kmh;
            float odometer_km;
            float tripmeter_km;
            bool operator!=(const Speedometer &rhs) {
                return (speed_kmh != rhs.speed_kmh || odometer_km != rhs.odometer_km
                        || tripmeter_km != rhs.tripmeter_km);
            }
        };

        struct Orientation {
            float yaw_deg;
            float pitch_deg;
            float roll_deg;
            bool operator!=(const Orientation &rhs) {
                return (yaw_deg != rhs.yaw_deg || pitch_deg != rhs.pitch_deg
                        || roll_deg != rhs.roll_deg);
            }
        };

        struct Location {
            DMS_t latitude;
            DMS_t longitude;
            bool operator!=(const Location &rhs) {
                return (latitude != rhs.latitude || longitude != rhs.longitude);
            }
        };

      private:
        bool config_received_from_vcu{false};
        bool config_save_requested{false};
        ServoState steering_servo;
        ServoState esc;
        TwoSpeedGearboxServoState gearbox_servo;
        Lighting lighting;
        bool turn_signal_free{false};
        bool turn_signal_left_cancelled{false};
        bool turn_signal_right_cancelled{false};
        RadioState radio;
        SensorState<Speedometer> speedometer;
        SensorState<Orientation> orientation;
        SensorState<Location> location;
        SensorState<int32_t> altitude;

        void checkSavedConfig();

      public:
        CarState(ServoConfigParams steering_servo_defaults, ServoConfigParams esc_defaults,
                 TwoSpeedGearboxServoConfigParams gearbox_servo_defaults,
                 RadioConfigParams radio_defaults);

        void setup();
        void spinOnce();

        bool isConfigAvailable();
        bool isCurrentConfigSaved();
        void requestConfigSave();
        bool isConfigSaveRequested();
        void markConfigSaved();

        void setSteeringInput(int16_t steering);
        int16_t getCurrentSteeringPosition();
        void setSteerLeftPulseWidth(uint16_t pulse_width);
        void setSteerCenterPulseWidth(uint16_t pulse_width);
        void setSteerRightPulseWidth(uint16_t pulse_width);
        void setSavedSteeringConfig(const ServoConfigParams &saved_config);
        bool isSteeringConfigAvailable();
        const ServoConfigParams &getSteeringConfig();
        bool isCurrentSteeringConfigSaved();
        void markSteeringConfigSaved();

        void setThrottleInput(int16_t throttle);
        int16_t getCurrentThrottlePosition();
        void setFullBackwardsPulseWidth(uint16_t pulse_width);
        void setStopPulseWidth(uint16_t pulse_width);
        void setFullForwardPulseWidth(uint16_t pulse_width);
        void setSavedThrottleConfig(const ServoConfigParams &saved_config);
        bool isThrottleConfigAvailable();
        const ServoConfigParams &getThrottleConfig();
        bool isCurrentThrottleConfigSaved();
        void markThrottleConfigSaved();

        void setCurrentGear(int16_t gear);
        int8_t getCurrentGear();
        void setGearPulseWidth(int8_t gear, uint16_t pulse_width);
        void setSavedGearboxConfig(const TwoSpeedGearboxServoConfigParams &saved_config,
                                   bool complete);
        bool isGearboxConfigAvailable();
        const TwoSpeedGearboxServoConfigParams &getGearboxConfig();
        bool isCurrentGearboxConfigSaved();
        void markGearboxConfigSaved();

        void setHmiRadioPower(uint8_t power_level);
        void setVcuRadioPower(uint8_t power_level);
        void setSavedRadioConfig(const RadioConfigParams &saved_config);
        bool isRadioConfigAvailable();
        const RadioConfigParams &getRadioConfig();
        bool isCurrentRadioConfigSaved();
        void markRadioConfigSaved();

        void setTurnSignalRight(bool state);
        void setTurnSignalLeft(bool state);
        void cancelTurnSignalsIfNeeded(int16_t steering);
        bool getTurnSignalRightCancelled(bool reset = true);
        bool getTurnSignalLeftCancelled(bool reset = true);
        void setDippedBeam(bool state);
        void setHighBeam(bool state);
        void setHazardFlashers(bool state);
        const Lighting &getRequestedLightingState();

        void setSpeedometer(const Speedometer &value, bool valid = true);
        bool isSpeedometerUpdated();
        SensorState<CarState::Speedometer>::Measurement getSpeedometer(bool reset_updated = true);
        void setOrientation(const Orientation &value, bool valid = true);
        bool isOrientationUpdated();
        SensorState<CarState::Orientation>::Measurement getOrientation(bool reset_updated = true);
        void setLocation(const Location &value, bool valid = true);
        bool isLocationUpdated();
        SensorState<CarState::Location>::Measurement getLocation(bool reset_updated = true);
        void setAltitude(int32_t value, bool valid = true);
        bool isAltitudeUpdated();
        SensorState<int32_t>::Measurement getAltitude(bool reset_updated = true);
    };

}  // namespace earth_rover

#endif