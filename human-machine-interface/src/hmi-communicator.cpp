#include "hmi-communicator.hpp"
#include "from-to-integral.hpp"


namespace earth_rover
{

  HmiCommunicator::HmiCommunicator(
    uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state):
    nrf24l01 {ce_pin, csn_pin},
    car_configuration {car_configuration},
    car_state {car_state}
  {
    ;
  }


  void HmiCommunicator::setup()
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


  void HmiCommunicator::spinOnce()
  {
    ;
  }


  bool HmiCommunicator::sendControlMessage()
  {
    static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    auto drive = car_state.getDriveInputs();
    auto car_id = car_configuration.getCarId();
    buffer[0] = car_id & 0xff;
    buffer[1] = ((car_id & 0x300) >> 2) + (to_integral(MessageType::Control) & 0x3f);
    buffer[2] = (drive.steering) & 0xff;
    buffer[3] = (drive.steering >> 8) & 0xff;
    buffer[4] = (drive.throttle) & 0xff;
    buffer[5] = (drive.throttle >> 8) & 0xff;
    buffer[6] = drive.gear;
    buffer[7] = 0;  // TODO: lighting!
    // buffer[8] = 0;  // Padding.
    // buffer[9] = 0;  // Padding.
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendMessage(uint8_t buffer[nrf24l01_payload_size])
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