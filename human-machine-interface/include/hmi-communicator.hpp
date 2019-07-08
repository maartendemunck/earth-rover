#ifndef __HMI_COMMUNICATOR__
#define __HMI_COMMUNICATOR__


#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.


namespace earth_rover
{

  template<uint8_t ce_pin, uint8_t csn_pin, uint8_t message_size>
  class HmiCommunicator
  {
    private:

      enum class MessageType: uint8_t { Control = 1 };
      RF24 nrf24l01;

    public:

      HmiCommunicator():
        nrf24l01(ce_pin, csn_pin)
      {
        ;
      }

      void setup()
      {
        // Setup nRF24L01+.
        nrf24l01.begin();
        nrf24l01.setPALevel(RF24_PA_LOW);
        nrf24l01.setDataRate(RF24_250KBPS);
        nrf24l01.setChannel(0x01);
        nrf24l01.setRetries(4, 10);
        nrf24l01.setAddressWidth(4);
        nrf24l01.setPayloadSize(message_size);
        nrf24l01.openWritingPipe((uint8_t *)"HMI0");
        nrf24l01.openReadingPipe(1, (uint8_t *)"VCU0");
        nrf24l01.startListening();
      }

      void spinOnce()
      {
        ;
      }

      bool sendControlMessage(int16_t steering, int16_t throttle)
      {
        static_assert(message_size >= 5, "control message requires a message payload of at least 5 bytes");
        uint8_t buffer[message_size];
        buffer[0] = to_integral(MessageType::Control);
        buffer[1] = (steering >> 8) & 0xff;
        buffer[2] = (steering) & 0xff;
        buffer[3] = (throttle >> 8) & 0xff;
        buffer[4] = (throttle) & 0xff;
        return sendMessage(buffer);
      }

    private:

      template<typename Enum>
      constexpr typename std::underlying_type<Enum>::type to_integral(Enum value)
      {
        return static_cast<typename std::underlying_type<Enum>::type>(value);
      }

      bool sendMessage(uint8_t buffer[message_size])
      {
        digitalWrite(LED_BUILTIN, 1);
        nrf24l01.stopListening();
        auto result = nrf24l01.write(buffer, message_size);
        nrf24l01.startListening();
        if(result)
        {
          digitalWrite(LED_BUILTIN, 0);
        }
        return result;
      }

  };

}

#endif