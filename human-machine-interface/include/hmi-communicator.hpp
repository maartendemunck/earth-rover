#ifndef __HMI_COMMUNICATOR__
#define __HMI_COMMUNICATOR__


#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.


namespace earth_rover
{

  class HmiCommunicator
  {
    private:

      enum class MessageType: uint8_t { Control = 1 };

      RF24 nrf24l01;
      static constexpr uint8_t nrf24l01_payload_size {8u};

    public:

      HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin);
      void setup();
      void spinOnce();
      bool sendControlMessage(int16_t steering, int16_t throttle);

    private:

      bool sendMessage(uint8_t buffer[]);

  };

}

#endif