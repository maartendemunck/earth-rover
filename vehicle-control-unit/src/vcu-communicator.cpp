#include "vcu-communicator.hpp"


namespace earth_rover
{

  VcuCommunicator::VcuCommunicator(
    uint8_t ce_pin,
    uint8_t csn_pin)
  :
    nrf24l01 {ce_pin, csn_pin}
  {
    ;
  }


  void
  VcuCommunicator::setup()
  {
    // Setup nRF24L01+.
    nrf24l01.begin();
    nrf24l01.setPALevel(RF24_PA_LOW);
    nrf24l01.setDataRate(RF24_250KBPS);
    nrf24l01.setChannel(0x01);
    nrf24l01.setRetries(4, 10);
    nrf24l01.setAddressWidth(4);
    nrf24l01.setPayloadSize(nrf24l01_payload_size);
    nrf24l01.openWritingPipe((uint8_t *)"VCU0");
    nrf24l01.openReadingPipe(1, (uint8_t *)"HMI0");
    nrf24l01.startListening();
  }


  void
  VcuCommunicator::spinOnce()
  {
    while(nrf24l01.available())
    {
      uint8_t buffer[nrf24l01_payload_size];
      nrf24l01.read(buffer, nrf24l01_payload_size);
      switch(static_cast<MessageType>(buffer[0]))
      {
        case MessageType::Control:
        {
          static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
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


  void
  VcuCommunicator::setControlMessageCallback(
    std::function<void(int16_t, int16_t)> callback)
  {
    control_message_callback = std::move(callback);
  }

}