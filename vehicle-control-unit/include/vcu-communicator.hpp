#ifndef __VCU_COMMUNICATOR__
#define __VCU_COMMUNICATOR__


#include <functional>
#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.


namespace earth_rover
{

  template<uint8_t ce_pin, uint8_t csn_pin, uint8_t message_size>
  class VcuCommunicator
  {
    private:

      enum class MessageType: uint8_t { Control = 1 };
      RF24 nrf24l01;
      std::function<void(int16_t, int16_t)> control_message_callback;

    public:

      VcuCommunicator():
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
        nrf24l01.openWritingPipe((uint8_t *)"VCU0");
        nrf24l01.openReadingPipe(1, (uint8_t *)"HMI0");
        nrf24l01.startListening();
      }

      void spinOnce()
      {
        while(nrf24l01.available())
        {
          uint8_t buffer[message_size];
          nrf24l01.read(buffer, message_size);
          switch(static_cast<MessageType>(buffer[0]))
          {
            case MessageType::Control:
            {
              static_assert(message_size >= 5, "control message requires a message payload of at least 5 bytes");
              int16_t steering = (buffer[1] << 8 ) | buffer[2];
              int16_t throttle = (buffer[3] << 8 ) | buffer[4];
              if(control_message_callback)
              {
                control_message_callback(steering, throttle);
              }
            } break;
            default:
            {
              Serial.printf("ignored received message with type %u", buffer[0]);
            } break;
          }
        }
      }

      void setControlMessageCallback(std::function<void(int16_t, int16_t)> callback)
      {
        control_message_callback = std::move(callback);
      }

    private:

      template<typename Enum>
      constexpr typename std::underlying_type<Enum>::type to_integral(Enum value)
      {
        return static_cast<typename std::underlying_type<Enum>::type>(value);
      }

  };

}

#endif