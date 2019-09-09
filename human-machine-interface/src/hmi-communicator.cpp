#include "hmi-communicator.hpp"
#include "from-to-integral.hpp"


namespace earth_rover
{

  constexpr uint8_t HmiCommunicator::nrf24l01_fhss_channels[];  // Initialised in header file.


  HmiCommunicator::HmiCommunicator(
    uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state):
    nrf24l01_device {ce_pin, csn_pin},
    nrf24l01_fhss_channel_index {0u},
    car_configuration {car_configuration},
    car_state {car_state}
  {
    ;
  }


  void HmiCommunicator::setup()
  {
    // Setup nRF24L01+.
    nrf24l01_device.begin();
    nrf24l01_device.setPALevel(RF24_PA_LOW);
    nrf24l01_device.setDataRate(RF24_250KBPS);
    nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
    nrf24l01_device.setRetries(4, 10);
    nrf24l01_device.setAddressWidth(4);
    nrf24l01_device.setPayloadSize(nrf24l01_payload_size);
    nrf24l01_device.openWritingPipe((uint8_t *)"HMI0");
    nrf24l01_device.openReadingPipe(1, (uint8_t *)"VCU0");
    nrf24l01_device.startListening();
  }


  void HmiCommunicator::spinOnce()
  {
    if(channel_changed == false && since_update >= update_interval - 10u)
    {
      changeChannel();
      channel_changed = true;
    }
    else if(channel_changed == true && since_update >= update_interval)
    {
      sendControlMessage();
      since_update -= update_interval;
      channel_changed = false;
    }
  }


  bool HmiCommunicator::sendControlMessage()
  {
    static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    auto drive = car_state.getDriveInputs();
    auto lighting = car_state.getLighting();
    buffer[0] = to_integral(MessageType::Control);
    buffer[1] = (drive.steering) & 0xff;
    buffer[2] = (drive.steering >> 8) & 0xff;
    buffer[3] = (drive.throttle) & 0xff;
    buffer[4] = (drive.throttle >> 8) & 0xff;
    buffer[5] = drive.gear;
    buffer[6] = lighting.turn_signal_right
                | (lighting.turn_signal_left << 1)
                | (lighting.dipped_beam << 2)
                | (lighting.high_beam << 3)
                | (lighting.hazard_flashers << 4);
    // buffer[7] = 0;  // Padding.
    // buffer[8] = 0;  // Padding.
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendMessage(uint8_t buffer[nrf24l01_payload_size])
  {
    digitalWrite(LED_BUILTIN, 1);
    nrf24l01_device.stopListening();
    auto result = nrf24l01_device.write(buffer, nrf24l01_payload_size);
    nrf24l01_device.startListening();
    if(result)
    {
      digitalWrite(LED_BUILTIN, 0);
    }
    else
    {
      // Serial.printf("F: %3d (%2d)\n", nrf24l01_fhss_channels[nrf24l01_fhss_channel_index], nrf24l01_fhss_channel_index);
    }
    
    return result;
  }


  void HmiCommunicator::changeChannel()
  {
    nrf24l01_fhss_channel_index = (nrf24l01_fhss_channel_index + 1)
                                  % (sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0]));
    nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
  }

}