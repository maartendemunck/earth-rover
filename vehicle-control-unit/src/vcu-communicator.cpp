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
      digitalWrite(LED_BUILTIN, 1);
      nrf24l01.read(buffer, nrf24l01_payload_size);
      auto message_timestamp = micros();
      auto message_processed = false;
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
          message_processed = true;
        } break;
        default:
        {
          message_processed = false;
        } break;
      }
      if(message_processed == true)
      {
        digitalWrite(LED_BUILTIN, 0);
        nrf24l01_timestamp_last_message = message_timestamp;
        timeout_callback_called = false;
      }
    }
    if(timeout_callback_called == false
       && (micros() - nrf24l01_timestamp_last_message) > timeout)
    {
      if(timeout_callback)
      {
        timeout_callback();
      }
      timeout_callback_called = true;
    }
  }


  void
  VcuCommunicator::setTimeoutCallback(
    std::function<void()> callback,
    uint32_t timeout_ms)
  {
    timeout_callback = std::move(callback);
    timeout = timeout_ms * 1000;
    timeout_callback_called = false;  // If we're in a timeout, call the new callback, even if we called the old one.
  }

  void
  VcuCommunicator::setControlMessageCallback(
    std::function<void(int16_t, int16_t)> callback)
  {
    control_message_callback = std::move(callback);
  }

}