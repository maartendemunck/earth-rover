// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__VCU__
#define __EARTH_ROVER__VCU__

#include "eeprom-configuration-database.hpp"
#include <cstdint>

namespace earth_rover {

    template <typename Steering_t, typename Powertrain_t, typename Lighting_t,
              typename PositionEncoder_t, typename Imu_t, typename Gps_t, typename Radio_t>
    class Vcu {

      private:
        enum class ConfigRecordType : uint8_t {
            Odometer,
            PositionEncoderCalibration,
            ImuCalibration,
            SteeringServoConfig,
            PowertrainConfig,
            RadioConfig
        };
        static constexpr uint8_t config_record_type_count{6u};
        static constexpr uint16_t config_record_size{32u};
        EepromConfigDatabase<config_record_size, config_record_type_count> config_database;

        Steering_t steering;
        Powertrain_t powertrain;
        Lighting_t lighting;
        PositionEncoder_t position_encoder;
        Imu_t imu;
        Gps_t gps;
        Radio_t radio;

        elapsedMillis since_last_control_message;
        bool timeout_handler_called;

        enum class RecordType : uint8_t {

        };

      public:
        Vcu(uint16_t eeprom_block_offset, uint16_t eeprom_block_size, Steering_t steering,
            Powertrain_t powertrain, Lighting_t lighting, PositionEncoder_t position_encoder,
            Imu_t imu, Gps_t gps, Radio_t radio)
            : config_database{eeprom_block_offset, eeprom_block_size}, steering{std::move(
                                                                           steering)},
              powertrain{std::move(powertrain)}, lighting{std::move(lighting)},
              position_encoder{std::move(position_encoder)}, imu{std::move(imu)},
              gps{std::move(gps)}, radio{std::move(radio)} {
            ;
        }

        void setup() {
            config_database.initialiseDatabase();
            config_database.loadObject(position_encoder, to_integral(ConfigRecordType::Odometer),
                                       false);
            config_database.loadObject(steering, to_integral(ConfigRecordType::SteeringServoConfig),
                                       true);
            steering.setup();
            config_database.loadObject(powertrain, to_integral(ConfigRecordType::PowertrainConfig),
                                       true);
            powertrain.setup();
            config_database.loadObject(radio, to_integral(ConfigRecordType::RadioConfig), true);
            lighting.setup();
            lighting.setStopLamps(true);
            lighting.setHazardFlashers(true);
            position_encoder.setup();
            config_database.loadObject(imu, to_integral(ConfigRecordType::ImuCalibration), true);
            imu.setup();
            gps.setup();
            // This is a safe state, no need to call the timeout handler before we receive the
            // first control message.
            timeout_handler_called = true;
        }

        void spinOnce() {
            if(since_last_control_message > 500u && !timeout_handler_called) {
                handleTimeout();
            }
            steering.spinOnce();
            powertrain.spinOnce();
            lighting.spinOnce();
            position_encoder.spinOnce();
            config_database.saveObjectIfRequired(position_encoder,
                                                 to_integral(ConfigRecordType::Odometer), true);
            imu.spinOnce();
            config_database.saveObjectIfRequired(imu, to_integral(ConfigRecordType::ImuCalibration),
                                                 true);
            gps.spinOnce();
        }

        void handleControlMessage(int16_t steering_angle, int16_t throttle_setting, int8_t gear,
                                  int8_t lighting_setting) {
            steering.setNormalizedSteeringAngle(steering_angle);
            powertrain.setNormalizedThrottleSetting(throttle_setting);
            powertrain.setGear(gear);
            lighting.setTurnSignalRight(lighting_setting & 0x01);
            lighting.setTurnSignalLeft(lighting_setting & 0x02);
            lighting.setDippedBeam(lighting_setting & 0x04);
            lighting.setHighBeam(lighting_setting & 0x08);
            lighting.setHazardFlashers(lighting_setting & 0x10);
            lighting.setStopLamps(throttle_setting > -50 && throttle_setting < 50);
            since_last_control_message = 0;
            timeout_handler_called = false;
        }

        int16_t getSpeed() const {
            return position_encoder.getSpeedInMeterPerHour();
        }

        uint64_t getOdometer() const {
            return position_encoder.getOdometerInMeter();
        }

        uint64_t getTripmeter() const {
            return position_encoder.getTripmeterInMeter();
        }

        EulerAngles_t getOrientation() {
            return imu.getOrientation();
        }

        bool isImuFullyCalibrated() {
            return imu.isFullyCalibrated();
        }

        gps_fix getGpsData() {
            return gps.getCurrentGpsFix();
        }

        SteeringServo::Config getSteeringServoConfig() {
            return steering.getConfig();
        }

        void configureSteeringServo(uint16_t pulse_width_left, uint16_t pulse_width_center,
                                    uint16_t pulse_width_right) {
            steering.configureSteeringServo(pulse_width_left, pulse_width_center,
                                            pulse_width_right);
        }

        Powertrain::Config getPowertrainConfig() {
            return powertrain.getConfig();
        }

        void configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop,
                          uint16_t pulse_width_forward) {
            powertrain.configureESC(pulse_width_reverse, pulse_width_stop, pulse_width_forward);
        }

        void configureGearboxServo(int8_t gear, uint16_t pulse_width) {
            powertrain.configureGearboxServo(gear, pulse_width);
        }

        void configureRadios(uint8_t hmi_radio_power_level, uint8_t vcu_radio_power_level) {
            radio.setHmiRadioPowerLevel(hmi_radio_power_level);
            radio.setVcuRadioPowerLevel(vcu_radio_power_level);
        }

        uint8_t getHmiRadioPowerLevel() {
            return radio.getHmiRadioPowerLevel();
        }

        uint8_t getVcuRadioPowerLevel() {
            return radio.getVcuRadioPowerLevel();
        }

        void saveConfig() {
            config_database.saveObjectIfRequired(
                steering, to_integral(ConfigRecordType::SteeringServoConfig), true);
            config_database.saveObjectIfRequired(
                powertrain, to_integral(ConfigRecordType::PowertrainConfig), true);
            config_database.saveObjectIfRequired(radio, to_integral(ConfigRecordType::RadioConfig),
                                                 true);
        }

      private:
        void handleTimeout() {
            // Stop car and turn stop lamps and hazard flashers on.
            powertrain.setNormalizedThrottleSetting(0);
            lighting.setStopLamps(true);
            lighting.setHazardFlashers(true);
            // Timeout handler called.
            timeout_handler_called = true;
        }
    };

}  // namespace earth_rover

#endif