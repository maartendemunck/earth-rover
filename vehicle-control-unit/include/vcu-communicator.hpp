#ifndef __VCU_COMMUNICATOR__
#define __VCU_COMMUNICATOR__


#include <functional>
#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.


namespace earth_rover
{

  class VcuCommunicator
  {
    private:

      enum class MessageType: uint8_t { Control = 0x00 };

      RF24 nrf24l01_device;
      static constexpr uint8_t nrf24l01_payload_size {10u};
      static constexpr uint8_t nrf24l01_fhss_channels[]
        { 76, 56, 28, 74, 16, 50, 62, 22, 90, 49, 10, 30, 32, 72,  2, 37,
          21, 36, 89, 73,  9, 68, 92, 13, 51,  7, 31, 61, 17, 84,  1, 86 };
      uint8_t nrf24l01_fhss_channel_index;
      static constexpr uint8_t update_interval {50u};
      elapsedMillis since_channel_change;
      bool channel_changed;
      uint32_t fhss_timeout {(sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0])) * update_interval};
      bool fhss_synced;

      elapsedMillis since_last_message;
      std::function<void()> timeout_callback;
      uint32_t fail_safe_timeout {1000u};
      bool timeout_callback_called;
      std::function<void(int16_t, int16_t, int8_t, uint8_t)> control_message_callback;

    public:

      VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin);

      void setup();
      void spinOnce();
      void setTimeoutCallback(std::function<void()> callback, uint32_t timeout_us);
      void setControlMessageCallback(std::function<void(int16_t, int16_t, int8_t, uint8_t)> callback);

    private:

      void changeChannel();
  };

}

#endif