// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "hmi-communicator.hpp"
#include "earth-rover-messages.hpp"
#include "from-to-integral.hpp"

namespace earth_rover {

    HmiCommunicator::HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarState &car_state)
        : PrimaryCommunicator{ce_pin, csn_pin}, car_state{car_state}, update_sequence_id{0u} {
        ;
    }

    void HmiCommunicator::spinOnce() {
        auto did_hop = hopToNextChannelIfNeeded();
        if(!did_hop && isSyncMessageNeeded()) {
            auto error = sendControlMessage();
            auto control_message_sent = !error;
            if(control_message_sent) {
                sendRequestStateMessages();
                synchronizeConfig();
            }
            ++update_sequence_id;
        }
        processIncomingMessages();
    }

    bool HmiCommunicator::sendControlMessage() {
        static_assert(message_payload_size >= 5,
                      "control message requires a payload size of at least 5 bytes");
        uint8_t buffer[message_payload_size];
        auto steering = car_state.getCurrentSteeringPosition();
        auto throttle = car_state.getCurrentThrottlePosition();
        auto gear = car_state.getCurrentGear();
        auto lighting = car_state.getRequestedLightingState();
        buffer[0] = to_integral(RequestMessageType::Control);
        buffer[1] = (steering)&0xff;
        buffer[2] = (steering >> 8) & 0xff;
        buffer[3] = (throttle)&0xff;
        buffer[4] = (throttle >> 8) & 0xff;
        buffer[5] = gear;
        buffer[6] = lighting.turn_signal_right | (lighting.turn_signal_left << 1)
                    | (lighting.dipped_beam << 2) | (lighting.high_beam << 3)
                    | (lighting.hazard_flashers << 4);
        return sendMessage(buffer, true);
    }

    bool HmiCommunicator::sendRequestStateMessages() {
        bool error{false};
        if(update_sequence_id % 6 == 0) {
            error |= sendRequestStateMessage(to_integral(RequestStateIDs::SpeedAndOdometer));
        }
        if(update_sequence_id % 6 == 2) {
            error |= sendRequestStateMessage(to_integral(RequestStateIDs::Orientation));
        }
        if(update_sequence_id % 12 == 4) {
            error |= sendRequestStateMessage(to_integral(RequestStateIDs::Location));
        }
        if(update_sequence_id % 12 == 10) {
            error |= sendRequestStateMessage(to_integral(RequestStateIDs::Altitude));
        }
        return error;
    }

    bool HmiCommunicator::sendRequestStateMessage(uint8_t requested_state) {
        static_assert(message_payload_size >= 2,
                      "the 'request state' message requires a payload size of at least 2 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::RequestState);
        buffer[1] = requested_state;
        return sendMessage(buffer, false);
    }

    bool HmiCommunicator::synchronizeConfig() {
        bool error{false};
        if(update_sequence_id % 2 == 1) {
            if(!car_state.isConfigAvailable()) {
                error |= requestNextConfigParameter();
            }
            else if(!car_state.isCurrentConfigSentToVcu()) {
                error |= sendNextConfigParameter();
            }
            else if(car_state.isConfigSaveByVcuRequested() && !error) {
                if(!(error |= sendSaveConfigMessage())) {
                    car_state.markConfigSavedByVcu();
                }
            }
        }
        return error;
    }

    bool HmiCommunicator::requestNextConfigParameter() {
        bool error{false};
        if(!car_state.isSteeringConfigAvailable()) {
            error |= sendRequestConfigMessage(to_integral(RequestConfigIDs::Steering));
        }
        else if(!car_state.isThrottleConfigAvailable()) {
            error |= sendRequestConfigMessage(to_integral(RequestConfigIDs::Throttle));
        }
        else if(!car_state.isGearboxConfigAvailable()) {
            error |= sendRequestConfigMessage(to_integral(RequestConfigIDs::Gearbox));
        }
        else if(!car_state.isRadioConfigAvailable()) {
            error |= sendRequestConfigMessage(to_integral(RequestConfigIDs::Radio));
        }
        return error;
    }

    bool HmiCommunicator::sendRequestConfigMessage(uint8_t requested_config) {
        static_assert(message_payload_size >= 2,
                      "the 'request config' message requires a payload size of at least 2 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::RequestConfig);
        buffer[1] = requested_config;
        return sendMessage(buffer, false);
    }

    bool HmiCommunicator::sendNextConfigParameter() {
        bool error{false};
        if(!car_state.isCurrentSteeringConfigSentToVcu()) {
            if(!(error |= sendCurrentSteeringConfig())) {
                car_state.markSteeringConfigSentToVcu();
            }
        }
        else if(!car_state.isCurrentThrottleConfigSentToVcu()) {
            if(!(error |= sendCurrentThrottleConfig())) {
                car_state.markThrottleConfigSentToVcu();
            }
        }
        else if(!car_state.isCurrentGearboxConfigSentToVcu()) {
            if(!(error |= sendCurrentGearboxConfig())) {
                car_state.markGearboxConfigSentToVcu();
            }
        }
        else if(!car_state.isCurrentRadioConfigSentToVcu()) {
            if(!(error |= sendCurrentRadioConfig())) {
                car_state.markRadioConfigSentToVcu();
            }
        }
        return error;
    }

    bool HmiCommunicator::sendCurrentSteeringConfig() {
        static_assert(message_payload_size >= 8, "the 'send steering config' message "
                                                 "requires a payload size of at least 8 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::ConfigureSteeringServo);
        auto config = car_state.getSteeringConfig();
        buffer[1] = (config.pulse_width_minimum & 0x00ff);
        buffer[2] = (config.pulse_width_minimum & 0xff00) >> 8;
        buffer[3] = (config.pulse_width_center & 0x00ff);
        buffer[4] = (config.pulse_width_center & 0xff00) >> 8;
        buffer[5] = (config.pulse_width_maximum & 0x00ff);
        buffer[6] = (config.pulse_width_maximum & 0xff00) >> 8;
        buffer[7] = config.input_channel;
        return sendMessage(buffer, false);
    }

    bool HmiCommunicator::sendCurrentThrottleConfig() {
        static_assert(message_payload_size >= 8, "the 'send throttle config' message "
                                                 "requires a payload size of at least 8 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::ConfigureEsc);
        auto config = car_state.getThrottleConfig();
        buffer[1] = (config.pulse_width_minimum & 0x00ff);
        buffer[2] = (config.pulse_width_minimum & 0xff00) >> 8;
        buffer[3] = (config.pulse_width_center & 0x00ff);
        buffer[4] = (config.pulse_width_center & 0xff00) >> 8;
        buffer[5] = (config.pulse_width_maximum & 0x00ff);
        buffer[6] = (config.pulse_width_maximum & 0xff00) >> 8;
        buffer[7] = config.input_channel;
        return sendMessage(buffer, false);
    }

    bool HmiCommunicator::sendCurrentGearboxConfig() {
        static_assert(
            message_payload_size >= 9,
            "the 'send gearbox config' message requires a payload size of at least 9 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::ConfigureGearboxServo);
        auto config = car_state.getGearboxConfig();
        buffer[1] = (2 << 4) | (0 << 3) | (0 << 0);  // 2 forward, no neutral, no reverse.
        buffer[2] = 1;
        buffer[3] = (config.getPulseWidthForGear(1) & 0x00ff);
        buffer[4] = (config.getPulseWidthForGear(1) & 0xff00) >> 8;
        buffer[5] = 2;
        buffer[6] = (config.getPulseWidthForGear(2) & 0x00ff);
        buffer[7] = (config.getPulseWidthForGear(2) & 0xff00) >> 8;
        buffer[8] = 0xff;
        auto error = sendMessage(buffer, false);
        Serial.printf("Sent gearbox message; error = %d\n", error);
        return error;
    }

    bool HmiCommunicator::sendCurrentRadioConfig() {
        static_assert(
            message_payload_size >= 8,
            "the 'send radio config' message requires a payload size of at least 3 bytes");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::ConfigureRadio);
        auto config = car_state.getRadioConfig();
        buffer[1] = config.tx_power;
        buffer[2] = config.rx_power;
        return sendMessage(buffer, false);
    }

    bool HmiCommunicator::sendSaveConfigMessage() {
        static_assert(message_payload_size >= 1,
                      "the 'save config' message requires a payload size of at least 1 byte");
        uint8_t buffer[message_payload_size];
        buffer[0] = to_integral(RequestMessageType::SaveConfig);
        return sendMessage(buffer, false);
    }

    void HmiCommunicator::processIncomingMessages() {
        uint8_t buffer[message_payload_size];
        while(!readIncomingMessage(buffer)) {
            switch(static_cast<ResponseMessageType>(buffer[0])) {
                case ResponseMessageType::Speedometer: {
                    processIncomingSpeedometerMessage(buffer);
                } break;
                case ResponseMessageType::Orientation: {
                    processIncomingOrientationMessage(buffer);
                } break;
                case ResponseMessageType::Location: {
                    processIncomingLocationMessage(buffer);
                } break;
                case ResponseMessageType::Altitude: {
                    processIncomingAltitudeMessage(buffer);
                } break;
                case ResponseMessageType::SteeringServoConfig: {
                    processIncomingSteeringConfigMessage(buffer);
                } break;
                case ResponseMessageType::EscConfig: {
                    processIncomingEscConfigMessage(buffer);
                } break;
                case ResponseMessageType::GearboxServoConfig: {
                    processIncomingGearboxConfigMessage(buffer);
                } break;
                case ResponseMessageType::RadioConfig: {
                    processIncomingRadioConfigMessage(buffer);
                } break;
            }
        }
    }

    void HmiCommunicator::processIncomingSpeedometerMessage(uint8_t buffer[message_payload_size]) {
        CarState::Speedometer speedometer;
        speedometer.speed_kmh = float(uint16_t(buffer[1] | (buffer[2] << 8))) / 1000.;
        speedometer.odometer_km
            = float(uint32_t(buffer[3] | (buffer[4] << 8) | (buffer[5] << 16))) / 1000.;
        speedometer.tripmeter_km
            = float(uint32_t(buffer[6] | (buffer[7] << 8) | (buffer[8] << 16))) / 1000.;
        car_state.setSpeedometer(speedometer);
    }

    void HmiCommunicator::processIncomingOrientationMessage(uint8_t buffer[message_payload_size]) {
        CarState::Orientation orientation;
        orientation.yaw_deg = float(uint16_t(buffer[1] | (buffer[2] << 8))) / 100.;
        orientation.pitch_deg = float(int16_t(buffer[3] | (buffer[4] << 8))) / 100.;
        orientation.roll_deg = float(int16_t(buffer[5] | (buffer[6] << 8))) / 100.;
        car_state.setOrientation(orientation);
    }

    void HmiCommunicator::processIncomingLocationMessage(uint8_t buffer[message_payload_size]) {
        CarState::Location location;
        location.latitude.From(int32_t(buffer[1]) | (int32_t(buffer[2]) << 8)
                               | (int32_t(buffer[3]) << 16) | (int32_t(buffer[4]) << 24));
        location.longitude.From(int32_t(buffer[5]) | (int32_t(buffer[6]) << 8)
                                | (int32_t(buffer[7]) << 16) | (int32_t(buffer[8]) << 24));
        car_state.setLocation(location, true);
    }

    void HmiCommunicator::processIncomingAltitudeMessage(uint8_t buffer[message_payload_size]) {
        car_state.setAltitude(
            int32_t(buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24)), true);
    }

    void
    HmiCommunicator::processIncomingSteeringConfigMessage(uint8_t buffer[message_payload_size]) {
        ServoConfigParams saved_config{
            uint8_t(buffer[7]),  // Input channel.
            uint16_t(buffer[1] | (buffer[2] << 8)),  // Pulse width to steer left.
            uint16_t(buffer[3] | (buffer[4] << 8)),  // Pulse width to drive straight ahead.
            uint16_t(buffer[5] | (buffer[6] << 8)),  // Pulse width to steer right.
            true  // Enforce pulse width limits.
        };
        car_state.setSavedSteeringConfig(saved_config);
    }

    void HmiCommunicator::processIncomingEscConfigMessage(uint8_t buffer[message_payload_size]) {
        ServoConfigParams saved_config{
            uint8_t(buffer[7]),  // Input channel.
            uint16_t(buffer[1] | (buffer[2] << 8)),  // Pulse width to drive backwards.
            uint16_t(buffer[3] | (buffer[4] << 8)),  // Pulse width to stop.
            uint16_t(buffer[5] | (buffer[6] << 8)),  // Pulse width to drive forward.
            true  // Enforce pulse width limits.
        };
        car_state.setSavedThrottleConfig(saved_config);
    }

    void
    HmiCommunicator::processIncomingGearboxConfigMessage(uint8_t buffer[message_payload_size]) {
        auto saved_config = car_state.getGearboxConfig();
        for(unsigned index = 0; index < 2; ++index) {
            int8_t gear = int8_t(buffer[2 + index * 3]);
            uint16_t pulse_width = uint16_t(buffer[3 + index * 3] | (buffer[4 + index * 3] << 8));
            if(gear >= 0 && gear <= 2) {
                saved_config.setPulseWidthForSingleGear(gear, pulse_width);
            }
        }
        bool complete = true;
        for(int8_t gear = -7; gear <= 15; ++gear) {
            if(saved_config.isGearAvailable(gear)
               && (saved_config.getPulseWidthForGear(gear) == 0
                   || saved_config.getPulseWidthForGear(gear) == 0xffff)) {
                complete = false;
            }
        }
        car_state.setSavedGearboxConfig(saved_config, complete);
    }

    void HmiCommunicator::processIncomingRadioConfigMessage(uint8_t buffer[message_payload_size]) {
        RadioConfigParams saved_config{buffer[1], buffer[2]};
        car_state.setSavedRadioConfig(saved_config);
    }

}  // namespace earth_rover