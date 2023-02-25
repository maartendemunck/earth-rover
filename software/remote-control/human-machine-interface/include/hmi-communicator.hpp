// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__HMI_COMMUNICATOR__
#define __EARTH_ROVER__HMI_COMMUNICATOR__

#include "car-state.hpp"
#include "communicator.hpp"
#include <cstdint>

namespace earth_rover {

    class HmiCommunicator : public PrimaryCommunicator<9u> {
      private:
        static constexpr uint8_t message_payload_size{9u};

        CarState &car_state;
        uint8_t update_sequence_id;

      public:
        HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarState &car_state);
        ~HmiCommunicator() = default;
        void spinOnce();

      private:
        bool sendControlMessage();
        bool sendRequestStateMessages();
        bool sendRequestStateMessage(uint8_t requested_state);
        bool synchronizeConfig();
        bool requestNextConfigParameter();
        bool sendRequestConfigMessage(uint8_t requested_config);
        bool sendNextConfigParameter();
        bool sendCurrentSteeringConfig();
        bool sendCurrentThrottleConfig();
        bool sendCurrentGearboxConfig();
        bool sendCurrentRadioConfig();
        bool sendSaveConfigMessage();
        void processIncomingMessages();
        void processIncomingSpeedometerMessage(uint8_t buffer[message_payload_size]);
        void processIncomingOrientationMessage(uint8_t buffer[message_payload_size]);
        void processIncomingLocationMessage(uint8_t buffer[message_payload_size]);
        void processIncomingAltitudeMessage(uint8_t buffer[message_payload_size]);
        void processIncomingSteeringConfigMessage(uint8_t buffer[message_payload_size]);
        void processIncomingEscConfigMessage(uint8_t buffer[message_payload_size]);
        void processIncomingGearboxConfigMessage(uint8_t buffer[message_payload_size]);
        void processIncomingRadioConfigMessage(uint8_t buffer[message_payload_size]);
    };

}  // namespace earth_rover

#endif