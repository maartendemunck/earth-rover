// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__VCU_COMMUNICATOR__
#define __EARTH_ROVER__VCU_COMMUNICATOR__

#include "communicator.hpp"
#include "earth-rover-messages.hpp"
#include "vcu.hpp"
#include <Arduino.h>
#include <functional>

namespace earth_rover {

    template <typename Vcu> class VcuCommunicator : public SecondaryCommunicator<9u> {
      private:
        static constexpr uint8_t nrf24l01_payload_size{9u};
        Vcu &vcu;

      public:
        VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin, Vcu &vcu)
            : SecondaryCommunicator{ce_pin, csn_pin}, vcu{vcu} {
            ;
        }

        void spinOnce() {
            hopToNextChannelIfNeeded();
            uint8_t buffer[nrf24l01_payload_size];
            while(!readIncomingMessage(buffer)) {
                auto message_processed = false;
                switch(static_cast<RequestMessageType>(buffer[0])) {
                    case RequestMessageType::Control:
                        handleControlMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::RequestState:
                        handleRequestStateMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::RequestConfig:
                        handleRequestConfigMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::ConfigureSteeringServo:
                        handleConfigureSteeringServoMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::ConfigureEsc:
                        handleConfigureEscMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::ConfigureGearboxServo:
                        handleConfigureGearboxServoMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::ConfigureRadio:
                        handleConfigureRadioMessage(buffer);
                        message_processed = true;
                        break;
                    case RequestMessageType::SaveConfig:
                        vcu.saveConfig();
                        message_processed = true;
                        break;
                }
                if(message_processed == true) {
                    digitalWrite(LED_BUILTIN, 0);
                }
            }
            checkSynchronisation();
        }

      private:
        bool handleControlMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 7,
                          "the 'control' message requires a payload size of at least 7 bytes");
            int16_t steering = buffer[1] | (buffer[2] << 8);
            int16_t throttle = buffer[3] | (buffer[4] << 8);
            int8_t gearbox = buffer[5];
            uint8_t lighting = buffer[6];
            markSyncMessageReception();
            vcu.handleControlMessage(steering, throttle, gearbox, lighting);
            return false;
        }

        bool handleRequestStateMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 2,
                          "the 'request state' message requires a payload size of at "
                          "least 2 bytes");
            uint8_t state_requested = buffer[1];
            bool error{false};
            if(state_requested & 0x01)  // Speed and odometer.
            {
                error |= sendSpeedometerMessage();
            }
            if(state_requested & 0x02)  // Orientation.
            {
                error |= sendOrientationMessage();
            }
            if(state_requested & 0x04)  // Location.
            {
                error |= sendLocationMessage();
            }
            if(state_requested & 0x08)  // Altitude.
            {
                error |= sendAltitudeMessage();
            }
            return error;
        }

        bool sendSpeedometerMessage() {
            static_assert(nrf24l01_payload_size >= 9,
                          "the 'speedometer' message requires a payload size of at least 9 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::Speedometer);
            auto speed = int16_t(vcu.getSpeed());
            buffer[1] = speed & 0x00ff;
            buffer[2] = (speed & 0xff00) >> 8;
            auto odometer = uint64_t(vcu.getOdometer()) & 0x00ffffff;
            buffer[3] = odometer & 0x0000ff;
            buffer[4] = (odometer & 0x00ff00) >> 8;
            buffer[5] = (odometer & 0xff0000) >> 16;
            auto tripmeter = uint64_t(vcu.getTripmeter()) & 0x00ffffff;
            buffer[6] = tripmeter & 0x0000ff;
            buffer[7] = (tripmeter & 0x00ff00) >> 8;
            buffer[8] = (tripmeter & 0xff0000) >> 16;
            return sendMessage(buffer);
        }

        bool sendOrientationMessage() {
            static_assert(nrf24l01_payload_size >= 8,
                          "the 'orientation' message requires a payload size of at least 8 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::Orientation);
            auto orientation = vcu.getOrientation();
            uint16_t yaw = uint16_t(orientation.yaw * 180. / M_PI * 100.);
            buffer[1] = yaw & 0x00ff;
            buffer[2] = (yaw & 0xff00) >> 8;
            int16_t pitch = int16_t(orientation.pitch * 180. / M_PI * 100.);
            buffer[3] = pitch & 0x00ff;
            buffer[4] = (pitch & 0xff00) >> 8;
            int16_t roll = int16_t(orientation.roll * 180. / M_PI * 100.);
            buffer[5] = roll & 0x00ff;
            buffer[6] = (roll & 0xff00) >> 8;
            buffer[7] = vcu.isImuFullyCalibrated() ? 0xff : 0x00;
            return sendMessage(buffer);
        }

        bool sendLocationMessage() {
            auto gps_data = vcu.getGpsData();
            if(gps_data.valid.location) {
                static_assert(nrf24l01_payload_size >= 9,
                              "the 'location' message requires a payload size of at least 9 bytes");
                uint8_t buffer[nrf24l01_payload_size];
                buffer[0] = static_cast<uint8_t>(ResponseMessageType::Location);
                const int32_t latitude = gps_data.latitudeL();
                buffer[1] = latitude & 0x000000ff;
                buffer[2] = (latitude & 0x0000ff00) >> 8;
                buffer[3] = (latitude & 0x00ff0000) >> 16;
                buffer[4] = (latitude & 0xff000000) >> 24;
                const int32_t longitude = gps_data.longitudeL();
                buffer[5] = longitude & 0x000000ff;
                buffer[6] = (longitude & 0x0000ff00) >> 8;
                buffer[7] = (longitude & 0x00ff0000) >> 16;
                buffer[8] = (longitude & 0xff000000) >> 24;
                return sendMessage(buffer);
            }
            else {
                return true;  // Ignore the location request if we don't have a valid location.
            }
        }

        bool sendAltitudeMessage() {
            auto gps_data = vcu.getGpsData();
            if(gps_data.valid.altitude) {
                static_assert(nrf24l01_payload_size >= 5,
                              "the 'altitude' message requires a payload size of at least 5 bytes");
                uint8_t buffer[nrf24l01_payload_size];
                buffer[0] = static_cast<uint8_t>(ResponseMessageType::Altitude);
                const int32_t altitude = gps_data.altitude_cm();
                buffer[1] = altitude & 0x000000ff;
                buffer[2] = (altitude & 0x0000ff00) >> 8;
                buffer[3] = (altitude & 0x00ff0000) >> 16;
                buffer[4] = (altitude & 0xff000000) >> 24;
                return sendMessage(buffer);
            }
            else {
                return true;  // Ignore the altitude request if we don't have a valid altitude.
            }
        }

        bool handleRequestConfigMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 2,
                          "the 'request config' message requires a payload size "
                          "of at least 2 bytes");
            uint8_t config_requested = buffer[1];
            bool error{false};
            if(config_requested & 0x01)  // Steering servo.
            {
                error |= sendSteeringServoConfigMessage();
            }
            if(config_requested & 0x02)  // ESC or throttle servo.
            {
                error |= sendEscConfigMessage();
            }
            if(config_requested & 0x04)  // Gearbox servo.
            {
                error |= sendGearboxServoConfigMessage();
            }
            if(config_requested & 0x80)  // Radio.
            {
                error |= sendRadioConfigMessage();
            }
            return error;
        }

        bool sendSteeringServoConfigMessage() {
            static_assert(nrf24l01_payload_size >= 8,
                          "the 'steering servo config' message requires a payload size of "
                          "at least 8 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::SteeringServoConfig);
            auto config = vcu.getSteeringServoConfig();
            buffer[1] = config.pulse_width_left & 0x00ff;
            buffer[2] = (config.pulse_width_left & 0xff00) >> 8;
            buffer[3] = config.pulse_width_center & 0x00ff;
            buffer[4] = (config.pulse_width_center & 0xff00) >> 8;
            buffer[5] = config.pulse_width_right & 0x00ff;
            buffer[6] = (config.pulse_width_right & 0xff00) >> 8;
            buffer[7] = 0xffu;
            return sendMessage(buffer);
        }

        bool sendEscConfigMessage() {
            static_assert(nrf24l01_payload_size >= 8,
                          "the 'ESC servo config' message requires a payload size of at "
                          "least 8 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::EscConfig);
            auto config = vcu.getPowertrainConfig();
            buffer[1] = config.esc.pulse_width_reverse & 0x00ff;
            buffer[2] = (config.esc.pulse_width_reverse & 0xff00) >> 8;
            buffer[3] = config.esc.pulse_width_stop & 0x00ff;
            buffer[4] = (config.esc.pulse_width_stop & 0xff00) >> 8;
            buffer[5] = config.esc.pulse_width_forward & 0x00ff;
            buffer[6] = (config.esc.pulse_width_forward & 0xff00) >> 8;
            buffer[7] = 0xffu;
            return sendMessage(buffer);
        }

        bool sendGearboxServoConfigMessage() {
            static_assert(nrf24l01_payload_size >= 8,
                          "the 'gearbox servo config' message requires a payload size of at "
                          "least 9 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            bool error{false};
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::GearboxServoConfig);
            auto config = vcu.getPowertrainConfig();
            buffer[1] = 0x28;  // No reverse gears, has neutral, two forward gears.
            buffer[2] = 0;
            buffer[3] = config.gearbox.pulse_width_neutral & 0x00ff;
            buffer[4] = (config.gearbox.pulse_width_neutral & 0xff00) >> 8;
            buffer[5] = 1;
            buffer[6] = config.gearbox.pulse_width_low & 0x00ff;
            buffer[7] = (config.gearbox.pulse_width_low & 0xff00) >> 8;
            buffer[8] = 0xffu;
            error |= sendMessage(buffer);
            buffer[2] = 2;
            buffer[3] = config.gearbox.pulse_width_high & 0x00ff;
            buffer[4] = (config.gearbox.pulse_width_high & 0xff00) >> 8;
            buffer[5] = 0x80;  // Unused
            buffer[6] = 0x00;
            buffer[7] = 0x00;
            error |= sendMessage(buffer);
            return error;
        }

        bool sendRadioConfigMessage() {
            static_assert(nrf24l01_payload_size >= 3,
                          "the 'radio config' message requires a payload size of at least 3 bytes");
            uint8_t buffer[nrf24l01_payload_size];
            buffer[0] = static_cast<uint8_t>(ResponseMessageType::RadioConfig);
            buffer[1] = vcu.getHmiRadioPowerLevel();
            buffer[2] = vcu.getVcuRadioPowerLevel();
            return sendMessage(buffer);
        }

        bool handleConfigureSteeringServoMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 7,
                          "the 'configure steering servo' message requires a payload "
                          "size of at least 7 bytes");
            auto pulse_width_left = buffer[1] | (buffer[2] << 8);
            auto pulse_width_center = buffer[3] | (buffer[4] << 8);
            auto pulse_width_right = buffer[5] | (buffer[6] << 8);
            vcu.configureSteeringServo(pulse_width_left, pulse_width_center, pulse_width_right);
            return false;
        }

        bool handleConfigureEscMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 7,
                          "the 'configure esc' message requires a payload size of at "
                          "least 7 bytes");
            auto pulse_width_backwards = buffer[1] | (buffer[2] << 8);
            auto pulse_width_stop = buffer[3] | (buffer[4] << 8);
            auto pulse_width_forward = buffer[5] | (buffer[6] << 8);
            vcu.configureESC(pulse_width_backwards, pulse_width_stop, pulse_width_forward);
            return false;
        }

        bool handleConfigureGearboxServoMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 9,
                          "the 'configure esc' message requires a payload size of at "
                          "least 9 bytes");
            for(unsigned index = 0; index < 2; ++index) {
                if(buffer[2 + 3 * index] != 0x80) {
                    vcu.configureGearboxServo(buffer[2 + 3 * index],
                                              buffer[3 + 3 * index] | (buffer[4 + 3 * index] << 8));
                }
            }
            return false;
        }

        bool handleConfigureRadioMessage(uint8_t buffer[nrf24l01_payload_size]) {
            static_assert(nrf24l01_payload_size >= 3,
                          "the 'configure esc' message requires a payload size of at "
                          "least 3 bytes");
            vcu.configureRadios(buffer[1], buffer[2]);
            setPowerLevel(buffer[2]);
            return false;
        }
    };

}  // namespace earth_rover

#endif