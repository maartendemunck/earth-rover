#include "hmi-communicator.hpp"
#include "to-integral.hpp"


namespace earth_rover
{

  HmiCommunicator::HmiCommunicator(
    uint8_t ce_pin,
    uint8_t csn_pin)
  :
    nrf24l01 {ce_pin, csn_pin}
  {
    ;
  }


  void
  HmiCommunicator::setup()
  {
    // Setup nRF24L01+.
    nrf24l01.begin();
    nrf24l01.setPALevel(RF24_PA_LOW);
    nrf24l01.setDataRate(RF24_250KBPS);
    nrf24l01.setChannel(0x01);
    nrf24l01.setRetries(4, 10);
    nrf24l01.setAddressWidth(4);
    nrf24l01.setPayloadSize(nrf24l01_payload_size);
    nrf24l01.openWritingPipe((uint8_t *)"HMI0");
    nrf24l01.openReadingPipe(1, (uint8_t *)"VCU0");
    nrf24l01.startListening();
  }


  void
  HmiCommunicator::spinOnce()
  {
    ;
  }


  bool
  HmiCommunicator::sendControlMessage(
    int16_t steering,
    int16_t throttle)
  {
    static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(MessageType::Control);
    buffer[1] = (steering >> 8) & 0xff;
    buffer[2] = (steering) & 0xff;
    buffer[3] = (throttle >> 8) & 0xff;
    buffer[4] = (throttle) & 0xff;
    return sendMessage(buffer);
  }


  bool
  HmiCommunicator::sendMessage(
    uint8_t buffer[nrf24l01_payload_size])
  {
    digitalWrite(LED_BUILTIN, 1);
    nrf24l01.stopListening();
    auto result = nrf24l01.write(buffer, nrf24l01_payload_size);
    nrf24l01.startListening();
    if(result)
    {
      digitalWrite(LED_BUILTIN, 0);
    }
    return result;
  }

}