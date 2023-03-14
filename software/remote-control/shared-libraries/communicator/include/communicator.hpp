// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__COMMUNICATOR__
#define __EARTH_ROVER__COMMUNICATOR__

#include <Arduino.h>
#include <RF24.h>
// RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#undef printf
#include <cstdint>

namespace earth_rover {

    template <uint8_t message_payload_size> class Communicator {
      public:
        enum class Direction : uint8_t { Downstream, Upstream };

      protected:
        static constexpr uint8_t fhss_channels[]{
            0, 8,  16, 24, 32, 40, 48, 56, 64, 72, 74, 66, 58, 50, 42, 34, 26, 18, 10, 2,
            4, 12, 20, 28, 36, 44, 52, 60, 68, 76, 78, 70, 62, 54, 46, 38, 30, 22, 14, 6};
        static constexpr uint8_t sync_message_interval{50u};
        static constexpr uint8_t downstream_pipe_name[]{'H', 'M', 'I', '0'};  // TODO: move to main.
        static constexpr uint8_t upstream_pipe_name[]{'V', 'C', 'U', '0'};  // TODO: move to main.

        RF24 device;
        const Direction direction;
        uint8_t power_level;
        uint8_t fhss_channel_index;
        elapsedMillis elapsed_time_since_channel_hop;
        bool channel_changed_since_sync_message;
        elapsedMillis elapsed_time_since_sync_message;

      public:
        Communicator(uint8_t ce_pin, uint8_t csn_pin, Direction direction)
            : device{ce_pin, csn_pin}, direction{direction}, power_level{0u},
              fhss_channel_index{0u}, channel_changed_since_sync_message{false} {
            ;
        }
        virtual ~Communicator() = default;

        virtual void setup() {
            device.begin();
            device.setPALevel(power_level);
            device.setDataRate(RF24_250KBPS);
            device.setChannel(fhss_channels[fhss_channel_index]);
            device.setRetries(4, 10);
            device.setAddressWidth(4);
            device.setPayloadSize(message_payload_size);
            if(direction == Direction::Downstream) {
                device.openWritingPipe(downstream_pipe_name);
                device.openReadingPipe(1, upstream_pipe_name);
            }
            else {
                device.openWritingPipe(upstream_pipe_name);
                device.openReadingPipe(1, downstream_pipe_name);
            }
            device.startListening();
        }

        bool setPowerLevel(uint8_t new_power_level) {
            if(new_power_level < 0 || new_power_level > 3) {
                return true;
            }
            if(new_power_level != power_level) {
                device.setPALevel(new_power_level);
                power_level = new_power_level;
            }
            return false;
        }

        uint8_t getPowerLevel() const {
            return power_level;
        }

        void hopToNextChannel() {
            fhss_channel_index
                = (fhss_channel_index + 1) % (sizeof(fhss_channels) / sizeof(fhss_channels[0]));
            device.setChannel(fhss_channels[fhss_channel_index]);
            elapsed_time_since_channel_hop = 0u;
            channel_changed_since_sync_message = true;
        }

        virtual bool hopToNextChannelIfNeeded() = 0;

        bool sendMessage(uint8_t message_buffer[message_payload_size]) {
            digitalWrite(LED_BUILTIN, 1);
            device.stopListening();
            auto success = device.write(message_buffer, message_payload_size);
            device.startListening();
            if(success) {
                digitalWrite(LED_BUILTIN, 0);
            }
            return !success;
        }

        bool sendMessage(uint8_t message_buffer[message_payload_size], bool is_sync_message) {
            auto error = sendMessage(message_buffer);
            if(is_sync_message) {
                elapsed_time_since_sync_message -= sync_message_interval;
                channel_changed_since_sync_message = false;
            }
            return error;
        }

        bool incomingMessageAvailable() {
            return device.available();
        }

        bool readIncomingMessage(uint8_t message_buffer[message_payload_size]) {
            if(incomingMessageAvailable()) {
                digitalWrite(LED_BUILTIN, 1);
                device.read(message_buffer, message_payload_size);
                digitalWrite(LED_BUILTIN, 0);
                return false;
            }
            else {
                return true;
            }
        }
    };

    template <uint8_t message_payload_size>
    constexpr uint8_t Communicator<message_payload_size>::fhss_channels[];

    template <uint8_t message_payload_size>
    constexpr uint8_t Communicator<message_payload_size>::downstream_pipe_name[];

    template <uint8_t message_payload_size>
    constexpr uint8_t Communicator<message_payload_size>::upstream_pipe_name[];

    template <uint8_t message_payload_size>
    class PrimaryCommunicator : public Communicator<message_payload_size> {
        using Base = Communicator<message_payload_size>;

      public:
        PrimaryCommunicator(uint8_t ce_pin, uint8_t csn_pin)
            : Base{ce_pin, csn_pin, Base::Direction::Downstream} {
            ;
        }
        virtual ~PrimaryCommunicator() = default;

        virtual bool hopToNextChannelIfNeeded() override {
            if(Base::channel_changed_since_sync_message == false
               && Base::elapsed_time_since_sync_message >= Base::sync_message_interval - 10u) {
                Base::hopToNextChannel();
                return true;
            }
            else {
                return false;
            }
        }

        bool isSyncMessageNeeded() {
            if(Base::channel_changed_since_sync_message == true
               && Base::elapsed_time_since_channel_hop >= 5u
               && Base::elapsed_time_since_sync_message >= Base::sync_message_interval) {
                return true;
            }
            else {
                return false;
            }
        }
    };

    template <uint8_t message_payload_size>
    class SecondaryCommunicator : public Communicator<message_payload_size> {
        using Base = Communicator<message_payload_size>;

      private:
        static constexpr uint32_t fhss_timeout{
            (sizeof(Base::fhss_channels) / sizeof(Base::fhss_channels[0]))
            * Base::sync_message_interval};
        bool fhss_is_synced_with_primary;

      public:
        SecondaryCommunicator(uint8_t ce_pin, uint8_t csn_pin)
            : Base{ce_pin, csn_pin, Base::Direction::Upstream}, fhss_is_synced_with_primary{false} {
            ;
        }
        virtual ~SecondaryCommunicator() = default;

        virtual bool hopToNextChannelIfNeeded() override {
            if((fhss_is_synced_with_primary == true
                && Base::elapsed_time_since_channel_hop >= Base::sync_message_interval)
               || (fhss_is_synced_with_primary == false
                   && Base::elapsed_time_since_channel_hop >= fhss_timeout)) {
                Base::hopToNextChannel();
                return true;
            }
            else {
                return false;
            }
        }

        void markSyncMessageReception() {
            Base::elapsed_time_since_channel_hop = 10u;
            Base::elapsed_time_since_sync_message = 0u;
            fhss_is_synced_with_primary = true;
        }

        bool checkSynchronisation() {
            if(Base::elapsed_time_since_sync_message > fhss_timeout) {
                fhss_is_synced_with_primary = false;
            }
            return fhss_is_synced_with_primary;
        }
    };

}  // namespace earth_rover

#endif