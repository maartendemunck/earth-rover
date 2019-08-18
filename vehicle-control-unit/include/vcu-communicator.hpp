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

      RF24 nrf24l01;
      static constexpr uint8_t nrf24l01_payload_size {10u};
      uint32_t nrf24l01_timestamp_last_message;
      std::function<void()> timeout_callback;
      uint32_t timeout;
      bool timeout_callback_called;
      std::function<void(int16_t, int16_t, int8_t, uint8_t)> control_message_callback;

    public:

      VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin);

      void setup();
      void spinOnce();
      void setTimeoutCallback(std::function<void(void)> callback, uint32_t timeout_us);
      void setControlMessageCallback(std::function<void(int16_t, int16_t, int8_t, uint8_t)> callback);

  };

}

#endif