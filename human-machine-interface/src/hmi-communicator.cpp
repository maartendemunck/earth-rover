#include "hmi-communicator.hpp"
#include "from-to-integral.hpp"


namespace earth_rover
{

  constexpr uint8_t HmiCommunicator::nrf24l01_fhss_channels[];  // Initialised in header file.


  HmiCommunicator::HmiCommunicator(
    uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state):
    nrf24l01_device {ce_pin, csn_pin},
    nrf24l01_fhss_channel_index {0u},
    update_sequence_id {0u},
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
      bool control_message_sent = sendControlMessage();
      since_update -= update_interval;
      if(control_message_sent && update_sequence_id % 6 == 0)
      {
        sendRequestStateMessage(0x01);
      }
      if(control_message_sent && update_sequence_id % 6 == 2)
      {
        sendRequestStateMessage(0x02);
      }
      if(control_message_sent && update_sequence_id % 12 == 4)
      {
        sendRequestStateMessage(0x04);
      }
      if(control_message_sent && update_sequence_id % 12 == 10)
      {
        sendRequestStateMessage(0x08);
      }
      ++ update_sequence_id;
      channel_changed = false;
    }
    while(nrf24l01_device.available())
    {
      uint8_t buffer[nrf24l01_payload_size];
      nrf24l01_device.read(buffer, nrf24l01_payload_size);
      switch(static_cast<ResponseMessageType>(buffer[0]))
      {
        case ResponseMessageType::Orientation:
        {
          CarState::Orientation orientation;
          orientation.yaw = float(uint16_t(buffer[1]) | (uint16_t(buffer[2]) << 8)) / 100.;
          orientation.pitch = float(int16_t(buffer[3]) | (int16_t(buffer[4]) << 8)) / 100.;
          orientation.roll = float(int16_t(buffer[5]) | (int16_t(buffer[6]) << 8)) / 100.;
          car_state.setOrientation(orientation);
        } break;
        case ResponseMessageType::Location:
        {
          CarState::Location location;
          location.latitude.From(int32_t(buffer[1]) | (int32_t(buffer[2]) << 8)
                                 | (int32_t(buffer[3]) << 16) | (int32_t(buffer[4]) << 24));
          location.longitude.From(int32_t(buffer[5]) | (int32_t(buffer[6]) << 8)
                                  | (int32_t(buffer[7]) << 16) | (int32_t(buffer[8]) << 24));
          car_state.setLocation(location, true);
        } break;
        case ResponseMessageType::Altitude:
        {
          car_state.setAltitude(int32_t(buffer[1]) | (int32_t(buffer[2]) << 8)
                                | (int32_t(buffer[3]) << 16) | (int32_t(buffer[4]) << 24), true);
        }
      }
    }
  }


  bool HmiCommunicator::sendControlMessage()
  {
    static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    auto drive = car_state.getDriveInputs();
    auto lighting = car_state.getLighting();
    buffer[0] = to_integral(RequestMessageType::Control);
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


  bool HmiCommunicator::sendRequestStateMessage(uint8_t requested_state)
  {
    static_assert(nrf24l01_payload_size >= 2,
                  "the 'request state' message requires a payload size of at least 2 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::RequestState);
    buffer[1] = requested_state;
    // buffer[2] = 0;  // Padding.
    // buffer[3] = 0;  // Padding.
    // buffer[4] = 0;  // Padding.
    // buffer[5] = 0;  // Padding.
    // buffer[6] = 0;  // Padding.
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